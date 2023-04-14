//
// Created by 钱晨 on 11/14/21.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "interface/led/led.h"
#include "shell.h"
#include "chassis_logic.h"
#include "can_motor_controller.h"
#include "can_motor_interface.h"
#include "remote_interpreter.h"
#include "hardware_conf.h"
// Other headers here

using namespace chibios_rt;
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

void startup_check_remote() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 50)) {
        if (not WITHIN_RECENT_TIME(Remote::last_update_time, 25)) {  // No signal in last 25 ms (normal interval 7 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(15);
    }
}

void startup_check_chassis_feedback() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        if (not WITHIN_RECENT_TIME(
                CANMotorIF::motor_feedback[CANMotorCFG::FR].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis FR offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(
                CANMotorIF::motor_feedback[CANMotorCFG::FL].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis FL offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(
                CANMotorIF::motor_feedback[CANMotorCFG::BL].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis BL offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(
                CANMotorIF::motor_feedback[CANMotorCFG::BR].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis BR offline.");
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}
/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */


// Thread to ...
class ChassisControl : public BaseStaticThread <512> {
private:
    void main() final {
        setName("Control");
        while (!shouldTerminate()) {
            if(Remote::rc.s1 == Remote::S_UP) {
                ChassisLG::set_mode(ChassisLG::FORCE_RELAX_MODE);
//                ChassisLG::set_target(0.0f, 0.0f);
//                ChassisLG::set_target_omega(0.0f);
                LOG("Response");
            } else {
                ChassisLG::set_mode(ChassisLG::CHASSIS_REF_MODE);
                ChassisLG::set_target(Remote::rc.ch2 * 1500.0f, Remote::rc.ch3 * 1000.0f);
                ChassisLG::set_target_omega(Remote::rc.ch0 * 50.0f);
            }
            sleep(TIME_MS2I(100));
        }
    }
} ControlThread;

class PrintParam: public BaseStaticThread<512>{
private:
    void main() final {
        setName("Param");
        while (!shouldTerminate()) {
            LOG("CAN Motor FeedBack:\n\r");
            LOG("FL: %d\n\r",CANMotorIF::motor_feedback[CANMotorCFG::FL].torque_current());
            LOG("FR: %d\n\r",CANMotorIF::motor_feedback[CANMotorCFG::FR].torque_current());
            LOG("LB: %d\n\r",CANMotorIF::motor_feedback[CANMotorCFG::BL].torque_current());
            LOG("LB: %d\n\r",CANMotorIF::motor_feedback[CANMotorCFG::BR].torque_current());
            sleep(TIME_MS2I(500));
        }
    }
} ParamThread;

int main() {
    halInit();
    System::init();

    LED::led_on(1);
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO+1);
    //CANMotorIF::init(&can1, &can2);
    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);
    chThdSleepMilliseconds(2000);
    startup_check_chassis_feedback();
    LED::led_on(2);
    Remote::start();
    startup_check_remote();
    LED::led_on(3);
    MecanumChassisSKD::init(HIGHPRIO-4,550.0f,500.0f,478.0f);
    ChassisLG::init(NORMALPRIO+4, NORMALPRIO+5, 180.0f);
    ChassisLG::set_mode(ChassisLG::CHASSIS_REF_MODE);
    ControlThread.start(NORMALPRIO + 6);
    LOG("Hello\n\r");


#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}
