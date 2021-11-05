//
// Created by 钱晨 on 11/4/21.
//

#include "haptic_logic.h"

int haptic_logic::current_threshold = 8000;
float haptic_logic::target_angle      = 0.0f;
float haptic_logic::velocity_threshold= 1.0f;
haptic_logic::back_driveability_thread haptic_logic::BackDriveabilityThd;
haptic_logic::button_switch_thread haptic_logic::ButtonSwitchThread;
LowPassFilteredValue haptic_logic::LowPassFilter[CANBUS_MOTOR_CFG::MOTOR_COUNT];
haptic_logic::mode_t haptic_logic::HAPTIC_DVC_MODE;

void haptic_logic::init(tprio_t PRIO, tprio_t FBPRIO) {
    BackDriveabilityThd.start(PRIO);
    ButtonSwitchThread.start(FBPRIO);
}

void haptic_logic::back_driveability_thread::main() {
    setName("Back_Drive_Ability_Thread");
    int target_current = 0;
    while(!shouldTerminate()) {
        for (auto &i: haptic_logic::LowPassFilter) {
            i.set_alpha(0.0001);
        }
        for (int i = 0; i < CANBUS_MOTOR_CFG::MOTOR_COUNT; i++) {
            switch (HAPTIC_DVC_MODE) {
                case torqueMode:
                    CANBUS_MOTOR_CFG::enable_a2v[i] = true;
                    CANBUS_MOTOR_CFG::enable_v2i[i] = CANBUS_MOTOR_CFG::DISABLED;
                    haptic_scheduler::set_target_current((CANBUS_MOTOR_CFG::motor_id_t) i, -

                    );
                    break;
                case velMode:
                    CANBUS_MOTOR_CFG::enable_a2v[i] = false;
                    CANBUS_MOTOR_CFG::enable_v2i[i] = CANBUS_MOTOR_CFG::WORKING;
                    haptic_scheduler::set_target_vel((CANBUS_MOTOR_CFG::motor_id_t) i,
                                                     can_motor_interface::motor_feedback[i].actual_velocity);
                    break;
                case angleMode:
                    CANBUS_MOTOR_CFG::enable_a2v[i] = true;
                    switch (CANBUS_MOTOR_CFG::enable_v2i[i]) {
                        case CANBUS_MOTOR_CFG::WORKING:
                            /// PID working, exceeded torque, disable PID and use linear torque.
                            if (!ABS_IN_RANGE(haptic_scheduler::get_torque_current((can_motor_interface::motor_id_t) i),
                                              current_threshold)) {
                                target_current = haptic_scheduler::get_torque_current(
                                        (can_motor_interface::motor_id_t) i);
                                CANBUS_MOTOR_CFG::enable_v2i[i] = CANBUS_MOTOR_CFG::DISABLED;
                                haptic_scheduler::set_target_current((CANBUS_MOTOR_CFG::motor_id_t) i,
                                                                     target_current);
                            }
                            break;
                        case CANBUS_MOTOR_CFG::DISABLED:
                            /// Speed decreased, smaller than threshold velocity
                            /// TODO: set to arbitrary torque during motion.
                            if (target_current * (int) can_motor_interface::motor_feedback[i].actual_velocity > 0) {
                                /// If current drive the motor backward, stop immediately.
                                haptic_scheduler::set_target_angle((CANBUS_MOTOR_CFG::motor_id_t) i,
                                                                   can_motor_interface::motor_feedback[i].accumulate_angle());
                                CANBUS_MOTOR_CFG::enable_v2i[i] = CANBUS_MOTOR_CFG::WORKING;
                            } else {
                                if (target_current > 100) {
                                    target_current -= 100;
                                } else if (target_current < -100) {
                                    target_current += 100;
                                } else {
                                    target_current = 0;
                                    CANBUS_MOTOR_CFG::enable_v2i[i] = CANBUS_MOTOR_CFG::FUSION;
                                    haptic_logic::LowPassFilter[i].set_alpha(0.0);
                                    haptic_logic::LowPassFilter[i].update(target_current);
                                    haptic_logic::LowPassFilter[i].set_alpha(0.0001);
                                }
                                /// Set motor angle to 0
                                haptic_scheduler::set_target_angle((CANBUS_MOTOR_CFG::motor_id_t) i,
                                                                   can_motor_interface::motor_feedback[i].accumulate_angle());
                                haptic_scheduler::set_target_current((CANBUS_MOTOR_CFG::motor_id_t) i, target_current);
                            }
                            break;
                        case CANBUS_MOTOR_CFG::FUSION:
                            /// If current conforms with current, let PID dominate current.
                            haptic_logic::LowPassFilter[i].update(
                                    (float) haptic_scheduler::get_PID_current((CANBUS_MOTOR_CFG::motor_id_t) i));
                            haptic_scheduler::set_target_current((CANBUS_MOTOR_CFG::motor_id_t) i,
                                                                 (int) haptic_logic::LowPassFilter[i].get());
                            if (ABS_IN_RANGE(haptic_logic::LowPassFilter[i].get() -
                                             haptic_scheduler::get_PID_current((CANBUS_MOTOR_CFG::motor_id_t) i),
                                             0.1)) {
                                haptic_scheduler::set_target_angle((CANBUS_MOTOR_CFG::motor_id_t) i,
                                                                   can_motor_interface::motor_feedback[i].accumulate_angle());
                                CANBUS_MOTOR_CFG::enable_v2i[i] = CANBUS_MOTOR_CFG::WORKING;
                            }
                            break;
                    }
                    break;
                case zeroVelMode:
                    CANBUS_MOTOR_CFG::enable_a2v[i] = false;
                    CANBUS_MOTOR_CFG::enable_v2i[i] = CANBUS_MOTOR_CFG::WORKING;
                    haptic_scheduler::set_target_vel((CANBUS_MOTOR_CFG::motor_id_t) i,
                                                     0.0f);
                    break;
            }
            // Not working properly.

            sleep(TIME_MS2I(10));
        }
    }
}

void haptic_logic::button_switch_thread::main() {
    setName("BtnSwtThd");
    while(!shouldTerminate()) {
        if (palReadPad(GPIOB, GPIOB_USER_BUTTON) == 1){
            if(HAPTIC_DVC_MODE < 3) {
                int temp = ((int)HAPTIC_DVC_MODE) + 1;
                HAPTIC_DVC_MODE = (haptic_logic::mode_t) temp;
            } else{
                HAPTIC_DVC_MODE = torqueMode;
            }
            LED::all_off();
            LED::led_on(HAPTIC_DVC_MODE);
        }
        sleep(TIME_MS2I(300));
    }
}
