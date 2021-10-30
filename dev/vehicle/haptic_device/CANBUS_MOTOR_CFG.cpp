//
// Created by 钱晨 on 10/29/21.
//

#include "CANBUS_MOTOR_CFG.h"

CANMotorBase CANBUS_MOTOR_CFG::CANMotorProfile[motor_id_t::MOTOR_COUNT] =
        {{CANMotorBase::can_channel_1, 0x201, CANMotorBase::M3508_without_deceleration, 0},
         {CANMotorBase::can_channel_1, 0x202, CANMotorBase::M3508_without_deceleration, 0}};