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
float SuspensionGimbalController::shoot_target_angle = 0; // in incontinuous mode, bullet loader stop if shoot_target_angle has been achieved
float SuspensionGimbalController::target_yaw_angle = 0.0f;
float SuspensionGimbalController::target_pitch_angle = 0.0f;


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
    shoot_target_angle = one_bullet_step * bullet_num + SuspensionGimbalIF::bullet_loader.actual_angle;
    continuous_shooting = false;
    set_shoot_mode(SuspensionGimbalIF::SHOOT);
}

void SuspensionGimbalController::set_shoot_mode(SuspensionGimbalIF::shoot_mode_t mode) {
    SuspensionGimbalIF::shoot_mode = mode;
}

void SuspensionGimbalController::set_target_signal() {
    // Set bullet loader current
    if (!continuous_shooting && SuspensionGimbalIF::bullet_loader.round_count * 360.0f + SuspensionGimbalIF::bullet_loader.actual_angle >= shoot_target_angle){
        set_shoot_mode(SuspensionGimbalIF::AWAIT);
    }

    if (SuspensionGimbalIF::shoot_mode == SuspensionGimbalIF::SHOOT) {
        SuspensionGimbalIF::bullet_loader.target_signal = (int16_t) BL_v_to_i.calc(SuspensionGimbalIF::bullet_loader.angular_velocity, bullet_loader_speed);
    } else {
        SuspensionGimbalIF::bullet_loader.target_signal = (int16_t) BL_v_to_i.calc(SuspensionGimbalIF::bullet_loader.angular_velocity, 0);
    }
    if (SuspensionGimbalIF::yaw.enabled)
        SuspensionGimbalIF::yaw.target_signal = (int16_t) yaw_v_to_i.calc(SuspensionGimbalIF::yaw.angular_velocity,
                yaw_angle_to_v.calc(SuspensionGimbalIF::yaw.actual_angle, target_yaw_angle));

    if (SuspensionGimbalIF::pitch.enabled)
        SuspensionGimbalIF::yaw.target_signal = (int16_t) pitch_v_to_i.calc(SuspensionGimbalIF::pitch.angular_velocity,
                pitch_angle_to_v.calc(SuspensionGimbalIF::pitch.actual_angle, target_pitch_angle));
}