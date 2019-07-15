//
// Created by liuzikai on 2019-04-12.
//

#include "sentry_chassis_interface.h"
#include "common_macro.h"

/* Parameters */

float SentryChassisIF::present_position;
float SentryChassisIF::present_velocity;
SentryChassisIF::motor_feedback_t SentryChassisIF::feedback[MOTOR_COUNT];
CANInterface *SentryChassisIF::can;

void SentryChassisIF::init(CANInterface *can_interface) {
    present_position = present_velocity = 0;
    feedback[MOTOR_LEFT].clear_position();
    feedback[MOTOR_RIGHT].clear_position();
    can = can_interface;

    can->register_callback(0x201, 0x202, process_feedback);
}

bool SentryChassisIF::send_currents() {

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

void SentryChassisIF::process_feedback(CANRxFrame const *rxmsg) {

    if (rxmsg->SID > 0x202 || rxmsg->SID < 0x201) return;

    unsigned id = (rxmsg->SID - 0x201);

    // Update new raw angle
    auto new_actual_angle_raw = (int16_t) (rxmsg->data8[0] << 8 | rxmsg->data8[1]);
    if (new_actual_angle_raw > 8191) return;

    // Process angular information
    int16_t angle_movement = new_actual_angle_raw - feedback[id].last_angle_raw;

    // Update last angle
    feedback[id].last_angle_raw = new_actual_angle_raw;

    // If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0(8192) point.
    if (angle_movement < -4096) {
        angle_movement += 8192;
    } else if (angle_movement > 4096) {
        angle_movement -= 8192;
    }

    // Update the actual data
    auto actual_rpm_raw = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);
    motor[id].actual_current_raw = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);

    // Update the actual angle
    motor[id].actual_angle += angle_movement;
    // modify the actual angle and update the round count when appropriate
    if (motor[id].actual_angle >= 8192) {
        //if the actual_angle is greater than 8192, then we can know that it has already turn a round in counterclockwise direction
        motor[id].actual_angle -= 8192;//set the angle to be within [0,8192)
        motor[id].round_count++;//round count increases 1
    }
    if (motor[id].actual_angle <= -8192) {
        // if the actual_angle is smaller than -8192, then we can know that it has already turn a round in clockwise direction
        motor[id].actual_angle += 8192;//set the angle to be within [0,8192)
        motor[id].round_count--;//round count decreases 1
    }


    // See the meaning of the motor decelerate ratio
    motor[id].actual_angular_velocity =
            actual_rpm_raw / CHASSIS_MOTOR_DECELERATE_RATIO * 360.0f / 60.0f;

    // Update the position and velocity

    // Count the rounds first, like 1.5 rounds, -20.7 rounds, etc
    // Then transform it to displacement by multiplying the DISPLACEMENT_PER_ROUND factor
    motor[id].motor_present_position =
            (motor[id].actual_angle + 8192.0f * motor[id].round_count) / 8192.0f * DISPLACEMENT_PER_ROUND /
            CHASSIS_MOTOR_DECELERATE_RATIO;
    // The unit of actual_angular_velocity is degrees/s, so we first translate it into r/s and then multiplying by DISPLACEMENT_PER_ROUND factor
    motor[id].motor_present_velocity = motor[id].actual_angular_velocity / 360.0f * DISPLACEMENT_PER_ROUND;

    present_position = (motor[MOTOR_RIGHT].motor_present_position + motor[MOTOR_LEFT].motor_present_position) / 2;
    present_velocity = (motor[MOTOR_RIGHT].motor_present_velocity + motor[MOTOR_LEFT].motor_present_velocity) / 2;
}