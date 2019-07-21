//
// Created by liuzikai on 2019-04-12.
//

#include "sentry_chassis_interface.h"
#include "common_macro.h"

SChassisIF::motor_feedback_t SChassisIF::feedback[MOTOR_COUNT];
int SChassisIF::target_current[MOTOR_COUNT];
CANInterface *SChassisIF::can;

void SChassisIF::init(CANInterface *can_interface) {

    feedback[MOTOR_LEFT].id = MOTOR_LEFT;
    feedback[MOTOR_LEFT].clear_position();

    feedback[MOTOR_RIGHT].id = MOTOR_RIGHT;
    feedback[MOTOR_RIGHT].clear_position();

    can = can_interface;
    can->register_callback(0x201, 0x202, process_feedback);
}

bool SChassisIF::send_currents() {
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
        ABS_CROP(motor[i].target_current, SENTRY_CHASSIS_MAX_CURRENT);
#endif
        txmsg.data8[i * 2] = (uint8_t) (target_current[i] >> 8);
        txmsg.data8[i * 2 + 1] = (uint8_t) target_current[i];
    }

    can->send_msg(&txmsg);
    return true;
}

void SChassisIF::process_feedback(CANRxFrame const *rxmsg) {

    if (rxmsg->SID > 0x202 || rxmsg->SID < 0x201) return;

    unsigned id = (rxmsg->SID - 0x201);

    /// Get new absolute angle value of motor
    uint16_t new_actual_angle_raw = (rxmsg->data8[0] << 8 | rxmsg->data8[1]);

    // Check whether this new raw angle is valid
    if (new_actual_angle_raw > 8191) return;

    /// Calculate the angle movement in raw data
    // KEY IDEA: add the change of angle to actual angle
    // We assume that the absolute value of the angle movement is smaller than 180 degrees (4096 of raw data)
    int angle_movement = (int) new_actual_angle_raw - (int) feedback[id].last_angle_raw;

    // Store new_actual_angle_raw for calculation of angle_movement next time
    feedback[id].last_angle_raw = new_actual_angle_raw;

    /// If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0(8192) point
    if (angle_movement < -4096) angle_movement += 8192;
    if (angle_movement > 4096) angle_movement -= 8192;

    // raw -> cm with deceleration ratio, / 8192 / (3591/187) * DISPLACEMENT_PER_ROUND
    feedback[id].present_position += angle_movement * 0.000006357f * DISPLACEMENT_PER_ROUND;

    // rpm -> cm/s with deceleration ratio, / 60 / (3591/187) * DISPLACEMENT_PER_ROUND
    feedback[id].present_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]))
                                    * 0.000867911f * DISPLACEMENT_PER_ROUND;

    feedback[id].last_update_time = SYSTIME;
}

float SChassisIF::present_velocity() {
    return (feedback[MOTOR_LEFT].present_velocity + feedback[MOTOR_RIGHT].present_velocity) / 2;
}

float SChassisIF::present_position() {
    return (feedback[MOTOR_LEFT].present_position + feedback[MOTOR_RIGHT].present_position) / 2;
}

void SChassisIF::clear_position() {
    feedback[MOTOR_LEFT].clear_position();
    feedback[MOTOR_RIGHT].clear_position();
}