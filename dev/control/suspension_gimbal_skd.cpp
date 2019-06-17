//
// Created by zhukerui on 2019/6/9.
//

#include "suspension_gimbal_skd.h"

PIDController SuspensionGimbalSKD::yaw_a2v_pid;
PIDController SuspensionGimbalSKD::yaw_v2i_pid;
PIDController SuspensionGimbalSKD::pitch_a2v_pid;
PIDController SuspensionGimbalSKD::pitch_v2i_pid;
PIDController SuspensionGimbalSKD::BL_v2i_pid;
SuspensionGimbalSKD::SuspensionGimbalThread SuspensionGimbalSKD::suspensionGimbalThread;
bool SuspensionGimbalSKD::continuous_shooting;


/**
 * Public Functions
 */

void SuspensionGimbalSKD::init() {
    yaw_a2v_pid.change_parameters(GIMBAL_YAW_A2V_PID_PARAMS);
    yaw_v2i_pid.change_parameters(GIMBAL_YAW_V2I_PID_PARAMS);
    pitch_a2v_pid.change_parameters(GIMBAL_PITCH_A2V_PID_PARAMS);
    pitch_v2i_pid.change_parameters(GIMBAL_PITCH_V2I_PID_PARAMS);
    BL_v2i_pid.change_parameters(GIMBAL_BL_V2I_PID_PARAMS);
    continuous_shooting = false;

    set_motor_enable(SuspensionGimbalIF::YAW_ID, false);
    set_motor_enable(SuspensionGimbalIF::PIT_ID, false);
    set_motor_enable(SuspensionGimbalIF::BULLET_LOADER_ID, false);
    set_shoot_mode(OFF);
}

void SuspensionGimbalSKD::set_front(SuspensionGimbalIF::motor_id_t motor_id) {
    if (motor_id == SuspensionGimbalIF::YAW_ID) SuspensionGimbalIF::yaw.reset_front_angle();
    else if (motor_id == SuspensionGimbalIF::PIT_ID) SuspensionGimbalIF::pitch.reset_front_angle();
    else if (motor_id == SuspensionGimbalIF::BULLET_LOADER_ID) SuspensionGimbalIF::bullet_loader.reset_front_angle();
}

void SuspensionGimbalSKD::set_motor_angle(SuspensionGimbalIF::motor_id_t motor_id, float target) {
    if (motor_id == SuspensionGimbalIF::YAW_ID) SuspensionGimbalIF::yaw.target_angle = target;
    else if (motor_id == SuspensionGimbalIF::PIT_ID) SuspensionGimbalIF::pitch.target_angle = target;
}

void SuspensionGimbalSKD::set_shoot_mode(shoot_mode_t mode) {
    SuspensionGimbalIF::shoot_mode = mode;
}

void SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::motor_id_t motor_id, bool status){
    if (motor_id == SuspensionGimbalIF::YAW_ID){
        SuspensionGimbalIF::yaw.enabled = status;
    } else if (motor_id == SuspensionGimbalIF::PIT_ID){
        SuspensionGimbalIF::pitch.enabled = status;
    } else if (motor_id == SuspensionGimbalIF::BULLET_LOADER_ID){
        SuspensionGimbalIF::bullet_loader.enabled = status;
    }
}

void SuspensionGimbalSKD::start_continuous_shooting() {
    continuous_shooting = true;
    set_shoot_mode(SHOOT);
}

void SuspensionGimbalSKD::stop_continuous_shooting() {
    set_shoot_mode(AWAIT);
}

void SuspensionGimbalSKD::start_incontinuous_shooting(int bullet_num) {
    SuspensionGimbalIF::bullet_loader.round_count = 0;
    SuspensionGimbalIF::bullet_loader.target_angle = one_bullet_step * bullet_num + SuspensionGimbalIF::bullet_loader.actual_angle;
    continuous_shooting = false;
    set_shoot_mode(SHOOT);
}

void SuspensionGimbalSKD::set_target_signal() {
    // Set bullet loader current
    if (!continuous_shooting && SuspensionGimbalIF::bullet_loader.angular_position >= SuspensionGimbalIF::bullet_loader.target_angle){
        set_shoot_mode(AWAIT);
    }
    if (SuspensionGimbalIF::shoot_mode == SHOOT) {
        SuspensionGimbalIF::bullet_loader.target_signal = (int16_t) BL_v2i_pid.calc(SuspensionGimbalIF::bullet_loader.angular_velocity, BULLET_LOADER_SPEED);
    } else {
        SuspensionGimbalIF::bullet_loader.target_signal = (int16_t) BL_v2i_pid.calc(SuspensionGimbalIF::bullet_loader.angular_velocity, 0);
    }
    // Set yaw voltage
    VAL_CROP(SuspensionGimbalIF::yaw.target_angle, MAX_YAW_ANGLE, MIN_YAW_ANGLE);
    if (SuspensionGimbalIF::yaw.enabled)
        SuspensionGimbalIF::yaw.target_signal = (int16_t) yaw_v2i_pid.calc(SuspensionGimbalIF::yaw.angular_velocity,
                yaw_a2v_pid.calc(SuspensionGimbalIF::yaw.angular_position, SuspensionGimbalIF::yaw.target_angle));

    // Set pitch voltage
    VAL_CROP(SuspensionGimbalIF::pitch.target_angle, MAX_PITCH_ANGLE, MIN_PITCH_ANGLE);
    if (SuspensionGimbalIF::pitch.enabled)
        SuspensionGimbalIF::pitch.target_signal = (int16_t) pitch_v2i_pid.calc(SuspensionGimbalIF::pitch.angular_velocity,
                pitch_a2v_pid.calc(SuspensionGimbalIF::pitch.angular_position, SuspensionGimbalIF::pitch.target_angle));
}

void SuspensionGimbalSKD::set_target_signal(SuspensionGimbalIF::motor_id_t motor, int16_t signal){
    if(motor == SuspensionGimbalIF::YAW_ID){
        SuspensionGimbalIF::yaw.target_signal = signal;
    } else if (motor == SuspensionGimbalIF::PIT_ID){
        SuspensionGimbalIF::pitch.target_signal = signal;
    } else if (motor == SuspensionGimbalIF::BULLET_LOADER_ID){
        SuspensionGimbalIF::bullet_loader.target_signal = signal;
    }
}

void SuspensionGimbalSKD::SuspensionGimbalThread::main() {
    setName("SentryGimbal");
    SuspensionGimbalSKD::init();
    while (!shouldTerminate()){
        set_target_signal();
        SuspensionGimbalIF::send_gimbal_currents();
        sleep(TIME_MS2I(10));
    }
}
