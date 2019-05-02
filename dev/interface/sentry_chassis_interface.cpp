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
        if(i == MOTOR_LEFT){
            txmsg.data8[i * 2] = (uint8_t) ((-motor[i].target_current) >> 8);
            txmsg.data8[i * 2 + 1] = (uint8_t) (-motor[i].target_current);
        } else{
            txmsg.data8[i * 2] = (uint8_t) ((motor[i].target_current) >> 8);
            txmsg.data8[i * 2 + 1] = (uint8_t) (motor[i].target_current);
        }
    }

    can->send_msg(&txmsg);
    return true;

}

void SentryChassis::process_feedback(CANRxFrame const*rxmsg) {

    if (rxmsg->SID > 0x202 || rxmsg->SID < 0x201) return;

    int motor_id = (int) (rxmsg->SID - 0x201);

    // Update new raw angle
    int16_t new_actual_angle_raw = (int16_t) (rxmsg->data8[0] << 8 | rxmsg->data8[1]);
    if (new_actual_angle_raw > 8191) return;

    // Process angular information
    int16_t angle_movement = new_actual_angle_raw - motor[motor_id].last_angle_raw;

    // If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0(8192) point.
    if (angle_movement < -4096) {
        angle_movement += 8192;
    } else if (angle_movement > 4096) {
        angle_movement -= 8192;
    }
    // Update the actual data
    motor[motor_id].actual_rpm_raw = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);
    motor[motor_id].actual_current_raw = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);
    motor[motor_id].actual_temperature_raw = rxmsg->data8[6];

    // Modify the data to the same direction, let the direction of the right wheel be the correct direction, and the left wheel is on the opposite
    if (motor_id == MOTOR_LEFT){
        angle_movement = - angle_movement;
        motor[motor_id].actual_rpm_raw = - motor[motor_id].actual_rpm_raw;
        motor[motor_id].actual_current_raw = - motor[motor_id].actual_current_raw;
    }
    // Update the actual angle
    motor[motor_id].actual_angle += angle_movement;
    // modify the actual angle and update the round count when appropriate
    if (motor[motor_id].actual_angle >= 8192) {
        //if the actual_angle is greater than 8192, then we can know that it has already turn a round in counterclockwise direction
        motor[motor_id].actual_angle -= 8192;//set the angle to be within [0,8192)
        motor[motor_id].round_count++;//round count increases 1
    }
    if (motor[motor_id].actual_angle <= -8192) {
        // if the actual_angle is smaller than -8192, then we can know that it has already turn a round in clockwise direction
        motor[motor_id].actual_angle += 8192;//set the angle to be within [0,8192)
        motor[motor_id].round_count--;//round count decreases 1
    }
    // Update last angle
    motor[motor_id].last_angle_raw = new_actual_angle_raw;

    // See the meaning of the motor decelerate ratio
    motor[motor_id].actual_angular_velocity =
            motor[motor_id].actual_rpm_raw / chassis_motor_decelerate_ratio * 360.0f / 60.0f;

}

void SentryChassis::init(CANInterface *can_interface) {
    can = can_interface;
    can->register_callback(0x201, 0x202, process_feedback);
}