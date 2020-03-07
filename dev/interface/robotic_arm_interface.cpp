//
// Created by Ye Anbang on 2019/2/2.
//
#include "robotic_arm_interface.h"


float RoboticArmIF::present_angle = 0;
float RoboticArmIF::present_velocity;
time_msecs_t RoboticArmIF::motor_last_update_time;
int16_t RoboticArmIF::motor_target_current;
uint16_t RoboticArmIF::motor_last_actual_angle_raw;
CANInterface *RoboticArmIF::can = nullptr;

void RoboticArmIF::init(CANInterface *can_interface) {
    motor_target_current = 0;
    can = can_interface;
    can->register_callback(0x205, 0x205, process_feedback);
    palSetPad(GPIOH, GPIOH_POWER2_CTRL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);
    chThdSleepMilliseconds(10);
    present_angle = 0;
}

void RoboticArmIF::process_feedback(CANRxFrame const *rxmsg) {

    if (rxmsg->SID != 0x205) return;

    uint16_t new_actual_angle_raw = rxmsg->data8[0] << 8 | rxmsg->data8[1];

    int angle_movement = (int) new_actual_angle_raw - (int) motor_last_actual_angle_raw;

    motor_last_actual_angle_raw = new_actual_angle_raw;

    // If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0(8192) point.
    if (angle_movement < -4096) {
        angle_movement += 8192;
    } else if (angle_movement > 4096) {
        angle_movement -= 8192;
    }

    present_angle = present_angle - angle_movement * 360.0f / 8192 / motor_decelerate_ratio;

    present_velocity = -((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) / motor_decelerate_ratio * 360.0f / 60.0f;

    motor_last_update_time = SYSTIME;
}

bool RoboticArmIF::send_current() {

    if (!can) return false;

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x1FF;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    // Fill in the can data frame with header 0x1FF and DATA[0], DATA[1]
    txmsg.data8[0] = (uint8_t) ((-motor_target_current) >> 8);
    txmsg.data8[1] = (uint8_t) (-motor_target_current);
    txmsg.data8[2] = txmsg.data8[3] = txmsg.data8[4] = txmsg.data8[5] = txmsg.data8[6] = txmsg.data8[7] = 0;

    can->send_msg(&txmsg);

    return true;

}