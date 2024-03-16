//
// Created by 钱晨 on 11/14/21.
//

#include "can_motor_config.h"

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_2, 0x202, CANMotorBase::M3508, 3572},//Front Left
        {CANMotorBase::can_channel_2, 0x201, CANMotorBase::M3508, 3572},//Front Right
        {CANMotorBase::can_channel_2, 0x204, CANMotorBase::M3508, 3572},//Back Right
        {CANMotorBase::can_channel_2, 0x203, CANMotorBase::M3508, 3572},//Back Left
        {CANMotorBase::can_channel_2, 0x208, CANMotorBase::GM6020, 70},//Yaw
        {CANMotorBase::can_channel_1, 0x205, CANMotorBase::GM6020, 3208},//PITCH 侧面的6020
        {CANMotorBase::can_channel_1, 0x201, CANMotorBase::M3508_without_deceleration, 3344},
        {CANMotorBase::can_channel_2, 0x206, CANMotorBase::M3508, 3572}, //Bullet Loader
        {CANMotorBase::can_channel_1, 0x203, CANMotorBase::M3508_without_deceleration, 3572},//FW_UP(左边)
        {CANMotorBase::can_channel_1, 0x204, CANMotorBase::M3508_without_deceleration, 3572}//FW_DOWN
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
        {15, 0.02f, 0.1, 70,  150},
        {40., 0.1f, 0.1f, 100, 200},
        {10, 0.0f, 0.2, 100, 500},
        {50, 0.0f, 0.18, 100, 250},//Bullet Loader temprarily to 2.5x to surpass the friction
        {10, 0.0f, 0.2, 100, 500},
        {10, 0.0f, 0.2, 100, 500},
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {35.0f,  1.0f, 1.0f, 5000.0f, 15000.0f},
        { 35.f, 1.f, 0.00f, 5000.0f, 15000.0f},
        {26.0f, 0.1f, 0.02f, 2000.0f, 6000.0f},
        {35.0f, 2.1f, 0.0f, 3000.0f, 16383.0f},//Bullet Loader
        {26.0f, 0.1f, 0.02f, 2000.0f, 6000.0f},
        {26.0f, 0.1f, 0.02f, 2000.0f, 6000.0f}
};

/// Will assigned by each scheduler.
bool CANMotorCFG::enable_a2v[MOTOR_COUNT] {false};
bool CANMotorCFG::enable_v2i[MOTOR_COUNT] {false};