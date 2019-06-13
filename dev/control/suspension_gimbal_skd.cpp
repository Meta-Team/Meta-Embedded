//
// Created by zhukerui on 2019/6/9.
//

#include "suspension_gimbal_skd.h"

PIDController SuspensionGimbalSKD::yaw_angle_to_v;
PIDController SuspensionGimbalSKD::yaw_v_to_i;
PIDController SuspensionGimbalSKD::pitch_angle_to_v;
PIDController SuspensionGimbalSKD::pitch_v_to_i;
PIDController SuspensionGimbalSKD::BL_v_to_i;
bool SuspensionGimbalSKD::continuous_shooting = false;


/**
 * Public Functions
 */

void SuspensionGimbalSKD::start_continuous_shooting() {
    continuous_shooting = true;
    set_shoot_mode(SHOOT);
}

void SuspensionGimbalSKD::stop_continuous_shooting() {
    set_shoot_mode(AWAIT);
}

void SuspensionGimbalSKD::start_incontinuous_shooting(int bullet_num) {
    bullet_loader.round_count = 0;
    bullet_loader.target_angle = one_bullet_step * bullet_num + bullet_loader.actual_angle;
    continuous_shooting = false;
    set_shoot_mode(SHOOT);
}

void SuspensionGimbalSKD::set_shoot_mode(shoot_mode_t mode) {
    shoot_mode = mode;
}

void SuspensionGimbalSKD::set_motor_enable(motor_id_t motor_id, bool status){
    if (motor_id == YAW_ID){
        yaw.enabled = status;
    } else if (motor_id == PIT_ID){
        pitch.enabled = status;
    } else if (motor_id == BULLET_LOADER_ID){
        bullet_loader.enabled = status;
    }
}

void SuspensionGimbalSKD::set_target_signal() {
    // Set bullet loader current
    if (!continuous_shooting && bullet_loader.round_count * 360.0f + bullet_loader.actual_angle >= bullet_loader.target_angle){
        set_shoot_mode(AWAIT);
    }
    if (shoot_mode == SHOOT) {
        bullet_loader.target_signal = (int16_t) BL_v_to_i.calc(bullet_loader.angular_velocity, BULLET_LOADER_SPEED);
    } else {
        bullet_loader.target_signal = (int16_t) BL_v_to_i.calc(bullet_loader.angular_velocity, 0);
    }
    // Set yaw voltage
    ABS_CROP(yaw.target_angle, MAX_YAW_ANGLE);
    if (yaw.enabled)
        yaw.target_signal = (int16_t) yaw_v_to_i.calc(yaw.angular_velocity,
                yaw_angle_to_v.calc(yaw.actual_angle, yaw.target_angle));

    // Set pitch voltage
    ABS_CROP(pitch.target_angle, MAX_PITCH_ANGLE);
    if (pitch.enabled)
        yaw.target_signal = (int16_t) pitch_v_to_i.calc(pitch.angular_velocity,
                pitch_angle_to_v.calc(pitch.actual_angle, pitch.target_angle));
}

void SuspensionGimbalSKD::set_target_signal(motor_id_t motor, int signal){
    if(motor == YAW_ID){
        yaw.target_signal = signal;
    } else if (motor == PIT_ID){
        pitch.target_signal = signal;
    } else if (motor == BULLET_LOADER_ID){
        bullet_loader.target_signal = signal;
    }
}

void SuspensionGimbalSKD::set_front(motor_id_t motor_id) {
    if (motor_id == YAW_ID) yaw.reset_front_angle();
    else if (motor_id == PIT_ID) pitch.reset_front_angle();
    else if (motor_id == BULLET_LOADER_ID) bullet_loader.reset_front_angle();
}
