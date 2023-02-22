//
// Created by Wu Feiyang on 2023/2/22.
//

#ifndef META_INFANTRY_CANBUS_MOTOR_CFG_H
#define META_INFANTRY_CANBUS_MOTOR_CFG_H

#include "can_motor_feedback.h"
#include "pid_controller.hpp"

class CANMotorCFG {
public:
    enum motor_id_t {
        FRONT,
        REAR,
        LEFT,
        RIHGT,
        BULLET_LOADER,
        FW_UP,
        FW_DOWN,
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
