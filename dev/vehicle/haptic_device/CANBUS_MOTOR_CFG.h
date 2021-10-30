//
// Created by 钱晨 on 10/29/21.
//

#ifndef META_INFANTRY_CANBUS_MOTOR_CFG_H
#define META_INFANTRY_CANBUS_MOTOR_CFG_H

#include "can_motor_feedback.h"
#include "pid_controller.hpp"

class CANBUS_MOTOR_CFG {
public:
    enum motor_id_t {
        YAW,
        MOTOR_COUNT
    };
    static CANMotorBase CANMotorProfile[motor_id_t::MOTOR_COUNT];

    // Parameters for double loop PID control.
    static PIDController::pid_params_t a2vParams[motor_id_t::MOTOR_COUNT];
    static PIDController::pid_params_t v2iParams[motor_id_t::MOTOR_COUNT];
};






#endif //META_INFANTRY_CANBUS_MOTOR_CFG_H
