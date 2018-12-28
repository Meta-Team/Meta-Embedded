//
// Created by Chuhao Feng on 2018/12/7.
//

#ifndef META_INFANTRY_SEND_CURRENTS_FUNCTIONS_H
#define META_INFANTRY_SEND_CURRENTS_FUNCTIONS_H

#define GIMBAL_MOTOR_YAW 0x205
#define GIMBAL_MOTOR_PIT 0x206
// need exact value to define
#define GIMBAL_MOTOR_MAX_CURRENT 10000
#define CHASSIS_MOTOR_MAX_CURRENT 10000

#include "ch.hpp"
#include "hal.h"
#include "common_macro.h"

class MotorCurrentSender{

    public:

    struct motor_t{
        int16_t target_current;
    };
    struct platform_t{
        motor_t motor[4];
    };
    platform_t chassis, gimbal;
    struct shoot_mechanism_t{
        int16_t stir_current;
    };
    shoot_mechanism_t shoot_mechanism;


    void send_chassis_currents(); // function for sending current signals to the chassis
    void send_gimbal_currents(); // function for sending current signals to the gimbal

};

#endif //META_INFANTRY_SEND_CURRENTS_FUNCTIONS_H
