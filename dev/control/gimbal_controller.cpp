//
// Created by liuzikai on 2019-01-05.
//

#include "gimbal_controller.h"

GimbalController::MotorController GimbalController::yaw(GimbalController::YAW_ID);
GimbalController::MotorController GimbalController::pitch(GimbalController::PIT_ID);
GimbalController::MotorController GimbalController::bullet_loader(GimbalController::BULLET_LOADER_ID);
GimbalController::FrictionWheelController GimbalController::frictionWheelController;
PIDController GimbalController::yaw_angle_to_v_pid;
PIDController GimbalController::yaw_v_to_i_pid;
PIDController GimbalController::pitch_angle_to_v_pid;
PIDController GimbalController::pitch_v_to_i_pid;
PIDController GimbalController::bullet_loader_angle_to_v_pid;
PIDController GimbalController::bullet_loader_v_to_i_pid;

bool GimbalController::start() {

}

float GimbalController::get_bullet_loader_current(float measured_angle, float measured_velocity, float target_angle) {
    if(target_angle < measured_angle){
        /**
         * Since the bullet loader could only move towards positive direction,
         * if the target angle is smaller than the present angle, then we regard that the loader needs to finish a round
         */
        measured_angle -= 360.0f;
    }
    float target_velocity;
    target_velocity = bullet_loader_angle_to_v_pid.calc(measured_angle, target_angle);
    return bullet_loader_v_to_i_pid.calc(measured_velocity, target_velocity);
}

float GimbalController::get_yaw_current(float measured_angle, float measured_velocity, float target_angle) {
    float target_velocity;
    target_velocity = yaw_angle_to_v_pid.calc(measured_angle, target_angle);
    return yaw_v_to_i_pid.calc(measured_velocity, target_velocity);
}

float GimbalController::get_pitch_current(float measured_angle, float measured_velocity, float target_angle) {
    float target_velocity;
    target_velocity = pitch_angle_to_v_pid.calc(measured_angle, target_angle);
    return pitch_v_to_i_pid.calc(measured_velocity, target_velocity);
}

bool GimbalController::change_angle_to_v_pid(GimbalController::motor_id_t id, float _kp, float _ki, float _kd,
                                             float _i_limit, float _out_limit) {
    switch (id){
        case YAW_ID:
            yaw_angle_to_v_pid.change_parameters(_kp, _ki, _kd, _i_limit, _out_limit);
            break;
        case PIT_ID:
            pitch_angle_to_v_pid.change_parameters(_kp, _ki, _kd, _i_limit, _out_limit);
            break;
        case BULLET_LOADER_ID:
            bullet_loader_angle_to_v_pid.change_parameters(_kp, _ki, _kd, _i_limit, _out_limit);
            break;
        default:
            return false;
    }
    return true;
}

bool
GimbalController::change_v_to_i_pid(GimbalController::motor_id_t id, float _kp, float _ki, float _kd, float _i_limit,
                                    float _out_limit) {
    switch (id){
        case YAW_ID:
            yaw_v_to_i_pid.change_parameters(_kp, _ki, _kd, _i_limit, _out_limit);
            break;
        case PIT_ID:
            pitch_v_to_i_pid.change_parameters(_kp, _ki, _kd, _i_limit, _out_limit);
            break;
        case BULLET_LOADER_ID:
            bullet_loader_v_to_i_pid.change_parameters(_kp, _ki, _kd, _i_limit, _out_limit);
            break;
        default:
            return false;
    }
    return true;
}
