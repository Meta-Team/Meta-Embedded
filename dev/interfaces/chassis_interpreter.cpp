//
// Created by 钱晨 on 2019-01-11.
//
#include "chassis_interpreter.h"
#include<math.h>

void chassis_interpreter::process_chassis_feedback(CANRxFrame *rxmsg) {
    int motor_id = rxmsg->SID - 0x201;
    uint16_t feedback_angle_orig = (rxmsg->data8[0] << 8 | rxmsg->data8[1]);
    chassis_inf.motor[motor_id].actual_angle = feedback_angle_orig / 8192;
    chassis_inf.motor[motor_id].actual_rpm = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);
    chassis_inf.motor[motor_id].actual_current = (rxmsg->data8[4] << 8 | rxmsg->data8[5]);
    chassis_inf.motor[motor_id].actual_temperature = rxmsg->data8[6];
    chassis_inf.motor[motor_id].actual_angular_velocity = chassis_inf.motor[motor_id].actual_rpm/60*2*M_PI;
}

