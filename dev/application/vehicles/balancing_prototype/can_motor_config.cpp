//
// Created by Tianyi Han on 2/21/2023.
//

#include "can_motor_config.h"

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_1, 0x205, CANMotorBase::GM6020, 3572},
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {60.0, 0.0f, 0.0, 1000.0, 30000.0},
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {2.0f,3.0f,0.0f,10000.0,30000.0},
};

bool CANMotorCFG::enable_a2v[MOTOR_COUNT] {
        true,
};

bool CANMotorCFG::enable_v2i[MOTOR_COUNT] {
        true,
};