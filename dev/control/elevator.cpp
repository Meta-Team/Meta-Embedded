//
// Created by liuzikai on 2019-05-17.
//

#include "elevator.h"

float Elevator::target_velocity[MOTOR_COUNT];
int Elevator::target_angle[MOTOR_COUNT];

PIDController Elevator::a2v_pid[MOTOR_COUNT];
PIDController Elevator::v2i_pid[MOTOR_COUNT];

void Elevator::change_pid_params(PIDControllerBase::pid_params_t a2v_params,
                                 PIDControllerBase::pid_params_t v2i_params) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        a2v_pid[i].change_parameters(a2v_params);
        v2i_pid[i].change_parameters(v2i_params);
    }
}

void Elevator::calc_a2v_(motor_id_t motor, float actual_angle_, float target_angle_) {
    target_velocity[motor] = a2v_pid[motor].calc(actual_angle_, target_angle_);
}

void Elevator::calc_v2i_(motor_id_t motor, float actual_velocity_, float target_velocity_) {
    target_velocity[motor] = target_velocity_;
    target_current[motor] = (int) v2i_pid[motor].calc(actual_velocity_, target_velocity_);
}

void Elevator::calc_elevator(motor_id_t motor) {
    calc_a2v_(motor, feedback[motor].accmulate_angle, target_angle[motor]);
    calc_v2i_(motor, feedback[motor].actual_velocity, target_velocity[motor]);
}

void Elevator::apply_front_position(float height) {
    target_angle[FL] = target_angle[FR] = (int) (height * HEIGHT_ANGLE_RATIO);
    calc_elevator(FL);
    calc_elevator(FR);
    feedback[FL].in_action = feedback[FR].in_action = true;
}

void Elevator::apply_back_position(float height) {
    target_angle[BL] = target_angle[BR] = (int) (height * HEIGHT_ANGLE_RATIO);
    calc_elevator(BL);
    calc_elevator(BR);
    feedback[BL].in_action = feedback[BR].in_action = true;
}