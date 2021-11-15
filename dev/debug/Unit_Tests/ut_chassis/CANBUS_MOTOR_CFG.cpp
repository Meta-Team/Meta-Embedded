//
// Created by 钱晨 on 11/14/21.
//

#include "CANBUS_MOTOR_CFG.h"

CANMotorBase CANBUS_MOTOR_CFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_1, 0x201, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_1, 0x202, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_1, 0x204, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_1, 0x203, CANMotorBase::M3508, 3572}
};

PIDController::pid_params_t CANBUS_MOTOR_CFG::a2vParams[MOTOR_COUNT] = {
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500}
};

PIDController::pid_params_t CANBUS_MOTOR_CFG::v2iParams[MOTOR_COUNT] = {
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0}
};

bool CANBUS_MOTOR_CFG::enable_a2v[MOTOR_COUNT] {
        false,
        false,
        false,
        false
};

CANBUS_MOTOR_CFG::v2i_PID_status_t CANBUS_MOTOR_CFG::enable_v2i[MOTOR_COUNT] {
        CANBUS_MOTOR_CFG::WORKING,
        CANBUS_MOTOR_CFG::WORKING,
        CANBUS_MOTOR_CFG::WORKING,
        CANBUS_MOTOR_CFG::WORKING
};