//
// Created by liuzikai on 2019-04-12.
//

#include "sentry_chassis_interface.h"
#include "common_macro.h"

SentryChassis::motor_t SentryChassis::motor[SentryChassis::MOTOR_COUNT];

CANInterface *SentryChassis::can = nullptr;

bool SentryChassis::send_currents() {

    if (!can) return false;

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x200;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    // Fill target currents
    for (int i = 0; i < MOTOR_COUNT; i++) {
#if SENTRY_CHASSIS_ENABLE_CLIP
        ABS_LIMIT(motor[i].target_current, SENTRY_CHASSIS_MAX_CURRENT);
#endif
        txmsg.data8[i * 2] = (uint8_t) (motor[i].target_current >> 8);
        txmsg.data8[i * 2 + 1] = (uint8_t) motor[i].target_current;
    }

    can->send_msg(&txmsg);
    return true;

}

void SentryChassis::process_feedback(CANRxFrame const*rxmsg) {

    if (rxmsg->SID > 0x202 || rxmsg->SID < 0x201) return;

    int motor_id = (int) (rxmsg->SID - 0x201);

    motor[motor_id].actual_angle_raw = (uint16_t) (rxmsg->data8[0] << 8 | rxmsg->data8[1]);
    motor[motor_id].actual_rpm_raw = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);
    motor[motor_id].actual_current_raw = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);
    motor[motor_id].actual_temperature_raw = rxmsg->data8[6];

    // See the meaning of the motor decelerate ratio
    motor[motor_id].actual_angular_velocity =
            motor[motor_id].actual_rpm_raw / chassis_motor_decelerate_ratio * 360.0f / 60.0f;

}

void SentryChassis::init(CANInterface *can_interface) {
    can = can_interface;
    can->register_callback(0x201, 0x202, process_feedback);
}