//
// Created by Chen Qian on 10/29/21.
//

#include "CANBUS_MOTOR_CFG.h"

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_1, 0x205, CANMotorBase::M3508_without_deceleration, 0},
        {CANMotorBase::can_channel_1, 0x202, CANMotorBase::M3508_without_deceleration, 0},
        {CANMotorBase::can_channel_1, 0x203, CANMotorBase::M3508_without_deceleration, 0},
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {20, 0.0f, 0.5, 100, 500},
        {20, 0.0f, 0.5, 100, 500},
        {20, 0.0f, 0.5, 100, 500}
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {25,0.4,0.000,13000.0,16000.0},
        {25,0.4,0.000,13000.0,16000.0},
        {25,0.4,0.000,13000.0,16000.0}
};

bool CANMotorCFG::enable_a2v[MOTOR_COUNT] {
    false,
    false,
    false
};

bool CANMotorCFG::enable_v2i[MOTOR_COUNT] {
        false,
        false,
        false
};