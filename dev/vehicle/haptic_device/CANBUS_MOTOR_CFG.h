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
        PITCH,
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
    static v2i_PID_status_t            enable_v2i[MOTOR_COUNT];
};

#endif //META_INFANTRY_CANBUS_MOTOR_CFG_H