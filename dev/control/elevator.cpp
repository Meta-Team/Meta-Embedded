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

void Elevator::calc_motor_(motor_id_t motor) {
    calc_a2v_(motor, feedback[motor].accmulate_angle, target_angle[motor]);
    calc_v2i_(motor, feedback[motor].actual_velocity, target_velocity[motor]);
}

void Elevator::calc_front(float height) {
    target_angle[FL] = target_angle[FR] = (int) (height * ANGLE_HEIGHT_RATIO);
    calc_motor_(FL);
    calc_motor_(FR);
}

void Elevator::calc_back(float height) {
    target_angle[BL] = target_angle[BR] = (int) (height * ANGLE_HEIGHT_RATIO);
    calc_motor_(BL);
    calc_motor_(BR);
}

float Elevator::get_front_height() {
    float fl_height = (float) feedback[FL].accmulate_angle / (float) ANGLE_HEIGHT_RATIO;
    float fr_height = (float) feedback[FR].accmulate_angle / (float) ANGLE_HEIGHT_RATIO;
    if (!ABS_IN_RANGE(fl_height - fr_height, UNBALANCE_LIMIT)) {
        StateHandler::raiseException(StateHandler::ELEVATOR_UNBALANCE);
    }
    return (fl_height + fr_height) / 2;
}

float Elevator::get_back_height() {
    float bl_height = (float) feedback[BL].accmulate_angle / (float) ANGLE_HEIGHT_RATIO;
    float br_height = (float) feedback[BR].accmulate_angle / (float) ANGLE_HEIGHT_RATIO;
    if (!ABS_IN_RANGE(bl_height - br_height, UNBALANCE_LIMIT)) {
        StateHandler::raiseException(StateHandler::ELEVATOR_UNBALANCE);
    }
    return (bl_height + br_height) / 2;
}

bool Elevator::motor_reach_target(motor_id_t motor) {
    return ABS_IN_RANGE(feedback[motor].accmulate_angle - target_angle[motor], STABLE_RANGE);
}