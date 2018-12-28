//
// Created by Chuhao Feng on 2018/12/7.
// Adapted from Zikai Liu
//

#include "send_currents_functions.h"

void MotorCurrentSender::send_chassis_currents() {

    // limit the maximum current of each of the motors
    ABS_LIMIT(chassis.motor[0].target_current, CHASSIS_MOTOR_MAX_CURRENT);
    ABS_LIMIT(chassis.motor[1].target_current, CHASSIS_MOTOR_MAX_CURRENT);
    ABS_LIMIT(chassis.motor[2].target_current, CHASSIS_MOTOR_MAX_CURRENT);
    ABS_LIMIT(chassis.motor[3].target_current, CHASSIS_MOTOR_MAX_CURRENT);

    // send current signals to proper location
    CANTxFrame txmsg_chassis; // create a new data structure only for chassis
    txmsg_chassis.IDE = CAN_IDE_STD; // variable declaration
    txmsg_chassis.SID = 0x200;
    txmsg_chassis.RTR = CAN_RTR_DATA;
    txmsg_chassis.DLC = 0x08;
    // motor 1
    txmsg_chassis.data8[0] = (uint8_t) (chassis.motor[0].target_current >> 8); // upper byte
    txmsg_chassis.data8[1] = (uint8_t) chassis.motor[0].target_current; // lower byte
    // motor 2
    txmsg_chassis.data8[2] = (uint8_t) (chassis.motor[1].target_current >> 8); // upper byte
    txmsg_chassis.data8[3] = (uint8_t) chassis.motor[1].target_current; // lower byte
    // motor 3
    txmsg_chassis.data8[4] = (uint8_t) (chassis.motor[2].target_current >> 8); // upper byte
    txmsg_chassis.data8[5] = (uint8_t) chassis.motor[2].target_current; // lower byte
    // motor 4
    txmsg_chassis.data8[6] = (uint8_t) (chassis.motor[3].target_current >> 8); //upper byte
    txmsg_chassis.data8[7] = (uint8_t) chassis.motor[3].target_current; // lower byte

    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg_chassis, TIME_MS2I(10));
}

void MotorCurrentSender::send_gimbal_currents() {

    // limit the maximum current of each of the motors
    ABS_LIMIT(gimbal.motor[GIMBAL_MOTOR_YAW].target_current, GIMBAL_MOTOR_MAX_CURRENT);
    ABS_LIMIT(gimbal.motor[GIMBAL_MOTOR_PIT].target_current, GIMBAL_MOTOR_MAX_CURRENT);

    int16_t zero_current = 0;

    // send current signals to proper location
    CANTxFrame txmsg_gimbal; // create a new data structure only for gimbal
    txmsg_gimbal.IDE = CAN_IDE_STD; // variable declaration
    txmsg_gimbal.SID = 0x1FF;
    txmsg_gimbal.RTR = CAN_RTR_DATA;
    txmsg_gimbal.DLC = 0x08;
    // YAW motor
    txmsg_gimbal.data8[0] = (uint8_t) (gimbal.motor[GIMBAL_MOTOR_YAW].target_current >> 8); //upper byte
    txmsg_gimbal.data8[1] = (uint8_t) gimbal.motor[GIMBAL_MOTOR_YAW].target_current; // lower byte
    // PITCH motor
    txmsg_gimbal.data8[2] = (uint8_t) (gimbal.motor[GIMBAL_MOTOR_PIT].target_current >> 8); // upper byte
    txmsg_gimbal.data8[3] = (uint8_t) gimbal.motor[GIMBAL_MOTOR_PIT].target_current; // lower byte
    // STIR motor
    txmsg_gimbal.data8[4] = (uint8_t) (shoot_mechanism.stir_current >> 8); // upper byte
    txmsg_gimbal.data8[5] = (uint8_t) shoot_mechanism.stir_current; // lower byte
    // current initialization
    txmsg_gimbal.data8[6] = (uint8_t) (zero_current >> 8); // upper byte
    txmsg_gimbal.data8[7] = (uint8_t) zero_current; // lower byte

    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg_gimbal, TIME_MS2I(10));
}
