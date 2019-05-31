//
// Created by liuzikai on 2019-01-05.
//

#include "gimbal.h"

PIDController Gimbal::a2v_pid[2];
PIDController Gimbal::v2i_pid[2];
float Gimbal::target_velocity[2];

void Gimbal::change_pid_params(PIDControllerBase::pid_params_t yaw_a2v_params,
                               PIDControllerBase::pid_params_t yaw_v2i_params,
                               PIDControllerBase::pid_params_t pitch_a2v_params,
                               PIDControllerBase::pid_params_t pitch_v2i_params) {
    a2v_pid[YAW].change_parameters(yaw_a2v_params);
    v2i_pid[YAW].change_parameters(yaw_v2i_params);

    a2v_pid[PITCH].change_parameters(pitch_a2v_params);
    v2i_pid[PITCH].change_parameters(pitch_v2i_params);
}

void Gimbal::calc_a2v_(GimbalInterface::motor_id_t id_, float actual_angle_, float target_angle_) {
    target_velocity[id_] = a2v_pid[id_].calc(actual_angle_, target_angle_);
}

void Gimbal::calc_v2i_(GimbalInterface::motor_id_t id_, float actual_velocity_, float target_velocity_) {
    target_velocity[id_] = target_velocity_;
    target_current[id_] = (int) v2i_pid[id_].calc(actual_velocity_, target_velocity_);
}

void Gimbal::calc_gimbal(float yaw_actual_velocity, float pitch_actual_velocity,
                         float yaw_target_angle, float pitch_target_angle) {

    calc_a2v_(YAW, feedback[YAW].actual_angle, yaw_target_angle);
    calc_v2i_(YAW, yaw_actual_velocity, target_velocity[YAW]);

    calc_a2v_(PITCH, feedback[PITCH].actual_angle, pitch_target_angle);
    calc_v2i_(PITCH, pitch_actual_velocity, target_velocity[PITCH]);

}