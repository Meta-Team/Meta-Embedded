//
// Created by 钱晨 on 11/14/21.
//

#include "can_motor_config.h"

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
    {CANMotorBase::can_channel_1, 0x207, CANMotorBase::M2006, 3572},
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
    {80, 0.0f, 0.0,  720, 720}
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
    {30.0f, 0.15f, 5.0f, 7000.0f, 10000.0f},
};

bool CANMotorCFG::enable_a2v[MOTOR_COUNT] {
    false,
};

bool CANMotorCFG::enable_v2i[MOTOR_COUNT] {
    false
};