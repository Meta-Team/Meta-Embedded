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
                ChassisLG::set_target(0.0f, 0.0f);
                ChassisLG::set_target_omega(0.0f);
            } else {
                ChassisLG::set_target(Remote::rc.ch2 * 1500.0f, Remote::rc.ch3 * 1000.0f);
                ChassisLG::set_target_omega(Remote::rc.ch0 * 50.0f);
            }
            sleep(TIME_MS2I(100));
        }
    }
} ControlThread;


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO+1);
    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);
    Remote::start();
    MecanumChassisSKD::init(251, 550.f,
                            500.f, 478.f);
    ChassisLG::init(NORMALPRIO+4, NORMALPRIO+5, 180.0f);
    ChassisLG::set_mode(ChassisLG::CHASSIS_REF_MODE);
    ControlThread.start(NORMALPRIO + 6);


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
