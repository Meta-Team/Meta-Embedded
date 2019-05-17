//
// Created by admin on 2019/1/16.
//

#include "elevator_interface.h"

ElevatorInterface::motor_feedback_t ElevatorInterface::feedback[MOTOR_COUNT];
int ElevatorInterface::target_current[ElevatorInterface::MOTOR_COUNT];
int ElevatorInterface::target_angle[ElevatorInterface::MOTOR_COUNT];
CANInterface *ElevatorInterface::can = nullptr;

bool ElevatorInterface::send_elevator_currents() {

    if (!can) return false;

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x1FF;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    // Fill target currents
    for (int i = 0; i < MOTOR_COUNT; i++) {
        txmsg.data8[i * 2] = (uint8_t) (target_current[i] >> 8);
        txmsg.data8[i * 2 + 1] = (uint8_t) target_current[i];
    }

    can->send_msg(&txmsg);
    return true;
}

void ElevatorInterface::process_elevator_feedback(CANRxFrame const *rxmsg) {

    chSysUnlock();  // --- Exit Critical Zone ---

    if (rxmsg->SID > 0x208 || rxmsg->SID < 0x205) return;

    int motor_id = (int) (rxmsg->SID - 0x205);

    auto new_angle_raw = (uint16_t) (rxmsg->data8[0] << 8 | rxmsg->data8[1]);
    feedback[motor_id].actual_rpm_raw = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);
    feedback[motor_id].actual_current_raw = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);
    feedback[motor_id].actual_temperature_raw = rxmsg->data8[6];

    int angle_movement = (int) feedback[motor_id].actual_angle_raw - (int) new_angle_raw;

    // If angle_movement is too extreme between two samples,
    // we grant that it's caused by moving over the 0(8192) point.
    if (angle_movement < -4096) {
        angle_movement += 8192;
    } else if (angle_movement > 4096) {
        angle_movement -= 8192;
    }

    feedback[motor_id].accmulate_angle += angle_movement;

    feedback[motor_id].actual_angle_raw = new_angle_raw;

    // See the meaning of the motor decelerate ratio
    feedback[motor_id].actual_velocity =
            feedback[motor_id].actual_rpm_raw / chassis_motor_decelerate_ratio * 360.0f / 60.0f;

    feedback[motor_id].last_update_time = SYSTIME;

    feedback[motor_id].in_action = !ABS_IN_RANGE(feedback[motor_id].accmulate_angle - target_angle[motor_id], STABLE_RANGE);

    chSysUnlock();  // --- Exit Critical Zone ---

}

void ElevatorInterface::init(CANInterface *can_interface) {
    can = can_interface;
    can->register_callback(0x205, 0x208, process_elevator_feedback);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        feedback[i].id = (motor_id_t) i;
        feedback[i].last_update_time = 0;
        feedback[i].clear_accmulate_angle();
        feedback[i].in_action = false;
        target_current[i] = 0;
        target_angle[i] = 0;
    }
}

void ElevatorInterface::motor_feedback_t::clear_accmulate_angle() {
    accmulate_angle = 0;
}

