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

#include <stdint.h>
#include "ch.hpp"
#include "hal.h"

class Send_Currents{

    public:
    // type definition
    typedef struct motor_t motor_t;
    typedef struct platform_t platform_t;
    typedef struct shoot_mechanism_t shoot_mechanism_t;

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

    CANTxFrame txmsg_chassis; // create a new data structure only for chassis
    txmsg_chassis.IDE = CAN_IDE_STD; // variable declaration
    txmsg_chassis.SID = 0x200;
    txmsg_chassis.RTR = CAN_RTR_DATA;
    txmsg_chassis.DLC = 0x08;

    void send_chassis_currents(); // function for sending current signals to the chassis

    CANTxFrame txmsg_gimbal; // create a new data structure only for gimbal
    txmsg_gimbal.IDE = CAN_IDE_STD; // variable declaration
    txmsg_gimbal.SID = 0x1FF;
    txmsg_gimbal.RTR = CAN_RTR_DATA;
    txmsg_gimbal.DLC = 0x08;

    void send_gimbal_currents(); // function for sending current signals to the gimbal

};

#endif //META_INFANTRY_SEND_CURRENTS_FUNCTIONS_H
