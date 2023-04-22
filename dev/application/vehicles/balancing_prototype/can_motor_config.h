//
// Created by Tianyi Han on 2/21/2023.
//

#ifndef META_EMBEDDED_CAN_MOTOR_CONFIG_H
#define META_EMBEDDED_CAN_MOTOR_CONFIG_H

#include "can_motor_feedback.h"
#include "pid_controller.hpp"

class CANMotorCFG {
public:
    enum motor_id_t {
        MOTOR1,
        MOTOR_COUNT
    };
    static CANMotorBase CANMotorProfile[MOTOR_COUNT];
    enum v2i_PID_status_t {
        DISABLED,
        WORKING,
        FUSION
    };
    // Parameters for double loop PID control.
    static PIDController::pid_params_t a2vParams [MOTOR_COUNT];
    static PIDController::pid_params_t v2iParams [MOTOR_COUNT];
    static bool                        enable_a2v[MOTOR_COUNT];
    static bool                        enable_v2i[MOTOR_COUNT];
};

#endif // META_EMBEDDED_CAN_MOTOR_CONFIG_H
