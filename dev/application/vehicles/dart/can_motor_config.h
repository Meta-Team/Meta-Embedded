//
// Created by 钱晨 on 11/14/21.
//

#ifndef META_INFANTRY_CANBUS_MOTOR_CFG_H
#define META_INFANTRY_CANBUS_MOTOR_CFG_H

#include "can_motor_feedback.h"
#include "pid_controller.hpp"

class CANMotorCFG {
public:
    enum motor_id_t {
        YAW,
        STORE_ENERGY_LEFT,
        STORE_ENERGY_RIGHT,
        TRIGGER_ADJUST,
        MOTOR_COUNT
    };
    static CANMotorBase CANMotorProfile[MOTOR_COUNT];

    // Parameters for double loop PID control.
    static PIDController::pid_params_t a2vParams [MOTOR_COUNT];
    static PIDController::pid_params_t v2iParams [MOTOR_COUNT];
    static bool                        enable_a2v[MOTOR_COUNT];
    static bool                        enable_v2i[MOTOR_COUNT];
};

#endif //META_INFANTRY_CANBUS_MOTOR_CFG_H
