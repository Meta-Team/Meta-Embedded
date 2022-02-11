//
// Created by Chen Qian on 11/4/21.
//

#include "haptic_logic.h"

int   HapticLG::current_threshold                      = 8000;
float HapticLG::target_angle[CANMotorCFG::MOTOR_COUNT] = {0.0f, 0.0f};
int   HapticLG::target_current[CANMotorCFG::MOTOR_COUNT] = {0,0};
float HapticLG::velocity_threshold                     = 1.0f;
HapticLG::BackDrivabilityThread HapticLG::back_drivability_thd;
HapticLG::ButtonSwitchThread    HapticLG::btn_switch_thd;
LowPassFilteredValue HapticLG::LowPassFilter[CANMotorCFG::MOTOR_COUNT];
HapticLG::mode_t HapticLG::haptic_device_mode = calibrateMode;
bool HapticLG::calibrated = false;

void HapticLG::init(tprio_t PRIO, tprio_t BTNPRIO) {
    back_drivability_thd.start(PRIO);
    btn_switch_thd.start(BTNPRIO);
}

bool HapticLG::device_calibrated() {
    return calibrated;
}

void HapticLG::BackDrivabilityThread::main() {
    setName("Back_Drive_Ability_Thread");
    time_msecs_t STUCK_STARTTIME = 0;
    long int SYS_STARTTIME = SYSTIME;
    while(!shouldTerminate()) {
        chSysLock();
        for (auto &i: HapticLG::LowPassFilter) {
            i.set_alpha(0.0001);
        }
        for (int i = 0; i < CANMotorCFG::MOTOR_COUNT; i++) {
            switch (haptic_device_mode) {
                case torqueMode:
                    CANMotorCFG::enable_a2v[i] = true;
                    CANMotorCFG::enable_v2i[i] = false;
                    CANMotorSKD::set_target_current((CANMotorCFG::motor_id_t) i, target_current[i]);
                    break;
                case calibrateMode:
                    CANMotorCFG::enable_a2v[i] = true;
                    CANMotorCFG::enable_v2i[i] = true;
                    CANMotorSKD::set_target_angle((CANMotorCFG::motor_id_t) i,
                                                  0.0f);
                    if(ABS_IN_RANGE(CANMotorInterface::motor_feedback[i].actual_velocity, 5.0) && // Velocity low
                      !ABS_IN_RANGE(CANMotorInterface::motor_feedback[i].accumulate_angle(), 15.0f) &&
                       WITHIN_RECENT_TIME(SYS_STARTTIME, CALIB_TIMEOUT_DELAY) && !calibrated) {
                        if(STUCK_STARTTIME == 0) {
                            STUCK_STARTTIME = SYSTIME;
                        } else if (!WITHIN_RECENT_TIME(STUCK_STARTTIME, CALIB_THRESHOLD_DELAY)) {
                            calibrated = true;
                            float zero_point = 0.0f;
                            if(i == 1) {
                                zero_point = SIGN(CANMotorInterface::motor_feedback[i].accumulate_angle()) > 0 ? 62.0f : -60.0f;
                            } else {
                                zero_point = SIGN(CANMotorInterface::motor_feedback[i].accumulate_angle()) * 45.0f;
                            }
                            CANMotorInterface::motor_feedback[i].actual_angle -= zero_point;
                        }
                    } else if (!WITHIN_RECENT_TIME(SYS_STARTTIME, CALIB_TIMEOUT_DELAY)){
                        calibrated = true;
                    }
                    break;
                case followMode:
                    CANMotorCFG::enable_a2v[i] = true;
                    CANMotorCFG::enable_v2i[i] = true;
                    CANMotorSKD::set_target_angle((CANMotorCFG::motor_id_t) i,
                                                  target_angle[i]);
                    break;
                case zeroVelMode:
                    CANMotorCFG::enable_a2v[i] = false;
                    CANMotorCFG::enable_v2i[i] = true;
                    CANMotorSKD::set_target_vel((CANMotorCFG::motor_id_t) i,
                                                0.0f);
                    break;
            }
        }
        chSysUnlock();
        sleep(TIME_MS2I(10));
    }
}

void HapticLG::ButtonSwitchThread::main() {
    setName("BtnSwtThd");
    while(!shouldTerminate()) {
        if (palReadPad(GPIOB, GPIOB_USER_BUTTON) == 1) {
            if(haptic_device_mode < 3) {
                int temp = ((int)haptic_device_mode) + 1;
                haptic_device_mode = (HapticLG::mode_t) temp;
            } else{
                haptic_device_mode = torqueMode;
            }
            LED::all_off();
            LED::led_on(haptic_device_mode);
        }
        sleep(TIME_MS2I(300));
    }
}
