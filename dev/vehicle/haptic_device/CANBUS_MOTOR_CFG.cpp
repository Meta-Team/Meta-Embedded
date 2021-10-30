//
// Created by 钱晨 on 10/29/21.
//

#include "CANBUS_MOTOR_CFG.h"

CANMotorBase CANBUS_MOTOR_CFG::CANMotorProfile[motor_id_t::MOTOR_COUNT] =
        {{CANMotorBase::can_channel_1, 0x205, CANMotorBase::M3508_without_deceleration, 0},
         {CANMotorBase::can_channel_1, 0x206, CANMotorBase::M3508_without_deceleration, 0},
         {CANMotorBase::can_channel_1, 0x207, CANMotorBase::M3508_without_deceleration, 0},
         {CANMotorBase::can_channel_2, 0x201, CANMotorBase::M3508_without_deceleration, 0},
         {CANMotorBase::can_channel_2, 0x202, CANMotorBase::M3508_without_deceleration, 0},
         {CANMotorBase::can_channel_2, 0x203, CANMotorBase::M3508_without_deceleration, 0},
         {CANMotorBase::can_channel_2, 0x204, CANMotorBase::M3508_without_deceleration, 0}
        };

PIDController::pid_params_t CANBUS_MOTOR_CFG::a2vParams[motor_id_t::MOTOR_COUNT] = {
        {150, 0.0f, 0, 500, 3000},
};

PIDController::pid_params_t CANBUS_MOTOR_CFG::v2iParams[motor_id_t::MOTOR_COUNT] = {
        {0.013,0.001,0.0001,1000.0,8000.0},
};