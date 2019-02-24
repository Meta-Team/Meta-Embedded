//
// Created by Ye Anbang on 2019/2/2.
//
#include "robotic_arm.h"
#include "common_macro.h"

RoboticArm::clamp_status_t RoboticArm::_clamp_status = RoboticArm::CLAMP_RELAX;
float RoboticArm::motor_accumulate_angle;
static int motor_target_current;
CANInterface *RoboticArm::can = nullptr;

float RoboticArm::get_motor_actual_angle() {
    return motor_accumulate_angle;
}

RoboticArm::clamp_status_t RoboticArm::get_clamp_status() {
    return _clamp_status;
}

void RoboticArm::clamp_action(RoboticArm::clamp_status_t target_status) {
    _clamp_status = target_status;
    palWritePad(GPIOH, GPIOH_ROBOTIC_ARM_CLAMP, _clamp_status);
}

void RoboticArm::init(CANInterface *can_interface) {
    can = can_interface;
    can->register_callback(0x205, 0x205, process_motor_feedback);
}

void RoboticArm::process_motor_feedback(CANRxFrame const *rxmsg) {
    if (rxmsg->SID != 0x205) return;

    uint16_t new_actual_angle_raw = rxmsg->data8[0] << 8 | rxmsg->data8[1];

    int angle_movement = (int) new_actual_angle_raw - (int) motor_last_actual_angle_raw;

    // TODO: figure out whether this actually work.
    if (angle_movement < -4096) {
        angle_movement += 8192;
    } else if (angle_movement > 4096) {
        angle_movement -= 8192;
    }

    motor_accumulate_angle = motor_accumulate_angle + angle_movement * 360.0f / 8192 / motor_decelerate_ratio;
}

void RoboticArm::reset_front_angle() {
    motor_accumulate_angle = 0;
}

void RoboticArm::set_motor_target_current(int target_current) {
    motor_target_current = target_current;
}

bool RoboticArm::send_motor_target_current() {

    if (!can) return false;

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x1FF;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    txmsg.data8[0] = (uint8_t) (motor_target_current >> 8);
    txmsg.data8[1] = (uint8_t) motor_target_current;
    txmsg.data8[2] = txmsg.data8[3] = txmsg.data8[4] = txmsg.data8[5] = txmsg.data8[6] = txmsg.data8[7] = 0;

    can->send_msg(&txmsg);
    return true;

}