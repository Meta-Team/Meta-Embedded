//
// Created by Quoke on 8/1/2022.
//

#include "ch.hpp"
#include "hal.h"

#include "interface/led/led.h"
#include "hardware_conf.h"
#include "debug/shell/shell.h"

#include "buzzer_scheduler.h"
// Other headers here
#include "can_motor_config.h"
#include "can_motor_controller.h"
#include "remote_interpreter.h"

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

class UTSentryChassisThread : public BaseStaticThread<512> {
    void main() final {
        setName("ut_sentry_chassis_thread");
        while (!shouldTerminate()) {
            if (Remote::rc.s1 == Remote::S_UP) { // Safe mode
                if(CANMotorCFG::enable_v2i[0]) {
                    for (auto &i: CANMotorCFG::enable_a2v) {
                        i = false;
                    }
                    for (int i = 0; i < CANMotorCFG::MOTOR_COUNT; i++) {
                        CANMotorController::set_target_vel((CANMotorCFG::motor_id_t) i, 0.0f);
                    }
                    sleep(TIME_MS2I(1000));
                }
                for (auto &i : CANMotorCFG::enable_v2i) {
                    i = false;
                }
                for (int i = 0; i < CANMotorCFG::MOTOR_COUNT; i++) {
                    CANMotorController::set_target_current((CANMotorCFG::motor_id_t)i, 0);
                }
                LED::led_off(1);
            } else if (Remote::rc.s1 == Remote::S_MIDDLE) {
                LED::led_on(1);
                for (auto &i : CANMotorCFG::enable_v2i) {
                    i = true;
                }
                for (int i = 0; i < CANMotorCFG::MOTOR_COUNT; i++) {
                    CANMotorController::set_target_vel((CANMotorCFG::motor_id_t)i, Remote::rc.ch0 * 500.0f);
                }
            }
            sleep(TIME_MS2I(5));
        }
    };
}ut_sentry_chassis_thread;

int main(void) {
    halInit();
    chibios_rt::System::init();

    LED::led_on(1);
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);

    LED::led_on(2);
    // Initiate CAN interface.
    can1.start(HIGHPRIO - 1);
    can2.start(HIGHPRIO - 2);
    chThdSleepMilliseconds(500);
    // Initiate CAN Motor interface
    chThdSleepMilliseconds(500);
    CANMotorController::start(HIGHPRIO - 3, HIGHPRIO - 4, &can1, &can2);
    chThdSleepMilliseconds(500);
    // Initiate remote controller
    Remote::start();
    LED::led_on(3);
    chThdSleepMilliseconds(500);
//    Shell::addCommands(ut_sentry_commands);

    // Start unit test thread.
    ut_sentry_chassis_thread.start(NORMALPRIO + 1);
    BuzzerSKD::init(NORMALPRIO);
    BuzzerSKD::play_sound(BuzzerSKD::sound_nyan_cat);


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