//
// Created by 钱晨 on 11/14/21.
//

#include "CANBUS_MOTOR_CFG.h"

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_2, 0x202, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x201, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x204, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x203, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x208, CANMotorBase::GM6020, 1994},
        {CANMotorBase::can_channel_1, 0x205, CANMotorBase::GM6020, 4600},
        {CANMotorBase::can_channel_1, 0x201, CANMotorBase::M3508_without_deceleration, 3344},
        {CANMotorBase::can_channel_2, 0x206, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_1, 0x203, CANMotorBase::M3508_without_deceleration, 3572},
        {CANMotorBase::can_channel_1, 0x204, CANMotorBase::M3508_without_deceleration, 3572}
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.1, 70,  90},
        {8.5, 0.0f, 0.1, 70, 90},
        {10, 0.0f, 0.2, 100, 500},
        {8.5, 0.0f, 0.18, 720, 720},
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {50.0f,  0.0f, 0.0f, 5000.0f, 15000.0f},
        { 50.0f, 0.0f, 0.00f, 5000.0f, 15000.0f},
        {26.0f, 0.1f, 0.02f, 2000.0f, 6000.0f},
        {35.0f, 2.1f, 0.0f, 3000.0f, 10000.0f},
        {26.0f, 0.1f, 0.02f, 2000.0f, 6000.0f},
        {26.0f, 0.1f, 0.02f, 2000.0f, 6000.0f}
};

/// Will assigned by each scheduler.
bool CANMotorCFG::enable_a2v[MOTOR_COUNT] {false};
bool CANMotorCFG::enable_v2i[MOTOR_COUNT] {false};