//
// Created by Kerui Zhu on 7/9/2019.
//

#include "engineer_chassis_interface.h"

EngineerChassisIF::motor_t EngineerChassisIF::motors[ENGINEER_CHASSIS_MOTOR_COUNT];
CANInterface *EngineerChassisIF::can = nullptr;

void EngineerChassisIF::init(CANInterface *can_interface) {
    can = can_interface;
    can->register_callback(0x201, 0x204, process_feedback);
    for (int i = 0; i < ENGINEER_CHASSIS_MOTOR_COUNT; i++) {
        motors[i].last_update_time = 0;
        motors[i].target_current = 0;
    }
}

bool EngineerChassisIF::send_currents() {

    if (!can) return false;

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x200;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    // Fill target currents
    for (int i = 0; i < ENGINEER_CHASSIS_MOTOR_COUNT; i++) {
#if CHASSIS_INTERFACE_ENABLE_CLIP
        ABS_CROP(target_current[i], CHASSIS_INTERFACE_MAX_CURRENT);
#endif
        txmsg.data8[i * 2] = (uint8_t) (motors[i].target_current >> 8);
        txmsg.data8[i * 2 + 1] = (uint8_t) motors[i].target_current;
    }

    can->send_msg(&txmsg);
    return true;

}

void EngineerChassisIF::process_feedback(CANRxFrame const *rxmsg) {

    if (rxmsg->SID > 0x204 || rxmsg->SID < 0x201) return;

    int motor_id = (rxmsg->SID - 0x201);

    motors[motor_id].actual_angle_raw = (uint16_t) (rxmsg->data8[0] << 8 | rxmsg->data8[1]);
    motors[motor_id].actual_rpm_raw = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);
    motors[motor_id].actual_current_raw = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);
    motors[motor_id].actual_temperature_raw = rxmsg->data8[6];

    // See the meaning of the motor decelerate ratio
    motors[motor_id].actual_velocity =
            motors[motor_id].actual_rpm_raw / chassis_motor_decelerate_ratio * 360.0f / 60.0f;

    motors[motor_id].last_update_time = SYSTIME;

}