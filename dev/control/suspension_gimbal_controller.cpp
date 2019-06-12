//
// Created by zhukerui on 2019/6/9.
//

#include "suspension_gimbal_controller.h"

PIDController SuspensionGimbalController::yaw_angle_to_v;
PIDController SuspensionGimbalController::yaw_v_to_i;
PIDController SuspensionGimbalController::pitch_angle_to_v;
PIDController SuspensionGimbalController::pitch_v_to_i;
PIDController SuspensionGimbalController::BL_v_to_i;
bool SuspensionGimbalController::continuous_shooting = false;


/**
 * Public Functions
 */

void SuspensionGimbalController::start_continuous_shooting() {
    continuous_shooting = true;
    set_shoot_mode(SuspensionGimbalIF::SHOOT);
}

void SuspensionGimbalController::stop_continuous_shooting() {
    set_shoot_mode(SuspensionGimbalIF::AWAIT);
}

void SuspensionGimbalController::start_incontinuous_shooting(int bullet_num) {
    SuspensionGimbalIF::bullet_loader.round_count = 0;
    SuspensionGimbalIF::bullet_loader.target_angle = one_bullet_step * bullet_num + SuspensionGimbalIF::bullet_loader.actual_angle;
    continuous_shooting = false;
    set_shoot_mode(SuspensionGimbalIF::SHOOT);
}

void SuspensionGimbalController::set_shoot_mode(SuspensionGimbalIF::shoot_mode_t mode) {
    SuspensionGimbalIF::shoot_mode = mode;
}

void SuspensionGimbalController::set_motor_enable(SuspensionGimbalIF::motor_id_t motor_id, bool status){
    if (motor_id == SuspensionGimbalIF::YAW_ID){
        SuspensionGimbalIF::yaw.enabled = status;
    } else if (motor_id == SuspensionGimbalIF::PIT_ID){
        SuspensionGimbalIF::pitch.enabled = status;
    } else if (motor_id == SuspensionGimbalIF::BULLET_LOADER_ID){
        SuspensionGimbalIF::bullet_loader.enabled = status;
    }
}

void SuspensionGimbalController::set_target_signal() {
    // Set bullet loader current
    if (!continuous_shooting && SuspensionGimbalIF::bullet_loader.round_count * 360.0f + SuspensionGimbalIF::bullet_loader.actual_angle >= SuspensionGimbalIF::bullet_loader.target_angle){
        set_shoot_mode(SuspensionGimbalIF::AWAIT);
    }
    if (SuspensionGimbalIF::shoot_mode == SuspensionGimbalIF::SHOOT) {
        SuspensionGimbalIF::bullet_loader.target_signal = (int16_t) BL_v_to_i.calc(SuspensionGimbalIF::bullet_loader.angular_velocity, BULLET_LOADER_SPEED);
    } else {
        SuspensionGimbalIF::bullet_loader.target_signal = (int16_t) BL_v_to_i.calc(SuspensionGimbalIF::bullet_loader.angular_velocity, 0);
    }
    // Set yaw voltage
    ABS_CROP(SuspensionGimbalIF::yaw.target_angle, MAX_YAW_ANGLE);
    if (SuspensionGimbalIF::yaw.enabled)
        SuspensionGimbalIF::yaw.target_signal = (int16_t) yaw_v_to_i.calc(SuspensionGimbalIF::yaw.angular_velocity,
                yaw_angle_to_v.calc(SuspensionGimbalIF::yaw.actual_angle, SuspensionGimbalIF::yaw.target_angle));

    // Set pitch voltage
    ABS_CROP(SuspensionGimbalIF::pitch.target_angle, MAX_PITCH_ANGLE);
    if (SuspensionGimbalIF::pitch.enabled)
        SuspensionGimbalIF::yaw.target_signal = (int16_t) pitch_v_to_i.calc(SuspensionGimbalIF::pitch.angular_velocity,
                pitch_angle_to_v.calc(SuspensionGimbalIF::pitch.actual_angle, SuspensionGimbalIF::pitch.target_angle));
}

void SuspensionGimbalController::set_target_signal(SuspensionGimbalIF::motor_id_t motor, int signal){
    if(motor == SuspensionGimbalIF::YAW_ID){
        SuspensionGimbalIF::yaw.target_signal = signal;
    } else if (motor == SuspensionGimbalIF::PIT_ID){
        SuspensionGimbalIF::pitch.target_signal = signal;
    } else if (motor == SuspensionGimbalIF::BULLET_LOADER_ID){
        SuspensionGimbalIF::bullet_loader.target_signal = signal;
    }
}

void SuspensionGimbalController::set_front(SuspensionGimbalIF::motor_id_t motor_id) {
    if (motor_id == SuspensionGimbalIF::YAW_ID) SuspensionGimbalIF::yaw.reset_front_angle();
    else if (motor_id == SuspensionGimbalIF::PIT_ID) SuspensionGimbalIF::pitch.reset_front_angle();
    else if (motor_id == SuspensionGimbalIF::BULLET_LOADER_ID) SuspensionGimbalIF::bullet_loader.reset_front_angle();
}
