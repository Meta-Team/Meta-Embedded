//
// Created by 钱晨 on 11/14/21.
//

#include "can_motor_config.h"

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_2, 0x202, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x201, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x204, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x203, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x205, CANMotorBase::GM6020, 2100},
        {CANMotorBase::can_channel_1, 0x205, CANMotorBase::GM6020, 3572},
        {CANMotorBase::can_channel_1, 0x207, CANMotorBase::M2006, 3572},
        {CANMotorBase::can_channel_1, 0x204, CANMotorBase::M3508_without_deceleration, 3572},
        {CANMotorBase::can_channel_1, 0x203, CANMotorBase::M3508_without_deceleration, 3572}
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {10, 0.0f, 0.2,  100, 500},
        {10, 0.0f, 0.2,  100, 500},
        {10, 0.0f, 0.2,  100, 500},
        {10, 0.0f, 0.2,  100, 500},
        {25, 0.0f, 1600, 000, 720},
        {30, 0.0f, 400,  000, 720},
        {20, 0.0f, 0.0,  000, 360},
        {10, 0.0f, 0.2,  100, 500},
        {10, 0.0f, 0.2,  100, 500},
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {25.0f,  0.15f, 0.0f, 5000.0f, 30000.0f},
        { 100.0f, 1.0f, 0.00f, 5000.0f, 30000.0f},
        {55.0f, 5.0f, 0.0f, 10000.0f, 10000.0f},
        {36.0f, 0.2f, 0.02f, 5000.0f, 10000.0f},
        {36.0f, 0.2f, 0.02f, 5000.0f, 10000.0f}
};

bool CANMotorCFG::enable_a2v[MOTOR_COUNT] {
        false,
        false,
        false,
        false,
        true,
        true,
        true,
        false,
        false
};

bool CANMotorCFG::enable_v2i[MOTOR_COUNT] {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
};