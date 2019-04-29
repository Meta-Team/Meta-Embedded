//
// Created by liuzikai on 2019-01-12.
//

#include "chassis_interface.h"
#include "common_macro.h"

ChassisInterface::motor_feedback_t ChassisInterface::feedback[CHASSIS_MOTOR_COUNT];
int ChassisInterface::target_current[ChassisInterface::CHASSIS_MOTOR_COUNT];
CANInterface *ChassisInterface::can = nullptr;

bool ChassisInterface::send_chassis_currents() {

    if (!can) return false;

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x200;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    // Fill target currents
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
#if CHASSIS_INTERFACE_ENABLE_CLIP
        ABS_LIMIT(target_current[i], CHASSIS_INTERFACE_MAX_CURRENT);
#endif
        txmsg.data8[i * 2] = (uint8_t) (target_current[i] >> 8);
        txmsg.data8[i * 2 + 1] = (uint8_t) target_current[i];
    }

    can->send_msg(&txmsg);
    return true;

}

void ChassisInterface::process_chassis_feedback(CANRxFrame const *rxmsg) {

    if (rxmsg->SID > 0x204 || rxmsg->SID < 0x201) return;

    int motor_id = (int) (rxmsg->SID - 0x201);

    feedback[motor_id].actual_angle_raw = (uint16_t) (rxmsg->data8[0] << 8 | rxmsg->data8[1]);
    feedback[motor_id].actual_rpm_raw = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);
    feedback[motor_id].actual_current_raw = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);
    feedback[motor_id].actual_temperature_raw = rxmsg->data8[6];

    // See the meaning of the motor decelerate ratio
    feedback[motor_id].actual_velocity =
            feedback[motor_id].actual_rpm_raw / chassis_motor_decelerate_ratio * 360.0f / 60.0f;

    feedback[motor_id].last_update_time = SYSTIME;

}

void ChassisInterface::init(CANInterface *can_interface) {
    can = can_interface;
    can->register_callback(0x201, 0x204, process_chassis_feedback);
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        feedback[i].id = (motor_id_t) i;
        feedback[i].last_update_time = 0;
        target_current[i] = 0;
    }
}