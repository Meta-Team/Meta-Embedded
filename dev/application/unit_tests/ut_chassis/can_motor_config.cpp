//
// Created by 钱晨 on 11/14/21.
//

#include "can_motor_config.h"\

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_1, 0x205, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_1, 0x201, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_1, 0x202, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_1, 0x204, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_1, 0x203, CANMotorBase::M3508, 3572}
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500}
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0}
};

bool CANMotorCFG::enable_a2v[MOTOR_COUNT] {
    false,
        false,
        false,
        false,
        false
};

bool CANMotorCFG::enable_v2i[MOTOR_COUNT] {
    false,
        false,
        false,
        false,
        false
};