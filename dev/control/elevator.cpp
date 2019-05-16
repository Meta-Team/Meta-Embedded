//
// Created by liuzikai on 2019-05-17.
//

#include "elevator.h"

float Elevator::target_velocity[MOTOR_COUNT];

PIDController Elevator::a2v_pid[MOTOR_COUNT];
PIDController Elevator::v2i_pid[MOTOR_COUNT];

void Elevator::change_pid_params(PIDControllerBase::pid_params_t a2v_params,
                                 PIDControllerBase::pid_params_t v2i_params) {
    for (int i = 0 ; i < MOTOR_COUNT; i++) {
        a2v_pid[i].change_parameters(a2v_params);
        v2i_pid[i].change_parameters(v2i_params);
    }
}

void Elevator::calc_a2v_(float actual_angle_, float target_angle_) {
    for (int i = 0 ; i < MOTOR_COUNT; i++) {
        target_velocity[i] = a2v_pid[i].calc(actual_angle_, target_angle_);
    }
}

void Elevator::calc_v2i_(float actual_velocity_, float target_velocity_) {
    for (int i = 0 ; i < MOTOR_COUNT; i++) {
        target_velocity[i] = target_velocity_;
        target_current[i] = (int) v2i_pid[i].calc(actual_velocity_, target_velocity_);
    }
}

void Elevator::calc_elevator(float height) {
    for (int i = 0 ; i < MOTOR_COUNT; i++) {
        calc_a2v_(feedback[i].accmulate_angle, height * HEIGHT_ANGLE_RATIO);
        calc_v2i_(feedback[i].actual_velocity, target_velocity[i]);
    }
}