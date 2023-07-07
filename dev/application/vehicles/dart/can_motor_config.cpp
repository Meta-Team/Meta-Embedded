//
// Created by Wu Feiyang on 7/6/23.
//

#include "can_motor_config.h"

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_1, 0x206, CANMotorBase::GM6020, 1419}
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {20, 0.0f, 500,  000, 550}
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {15.0f,0.0f,5.0f,5000,30000.0},

};

bool CANMotorCFG::enable_a2v[MOTOR_COUNT] {false};

bool CANMotorCFG::enable_v2i[MOTOR_COUNT] {false};