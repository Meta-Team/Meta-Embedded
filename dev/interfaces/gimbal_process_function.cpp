//
// Created by admin on 2018/12/8.
//
#include "gimbal_process_function.h"


void gimbal_process::process_gimbal_feedback(CANRxFrame *rxmsg) {

    int motor_id = rxmsg->SID - 0x205;//get the motor id (YAW)
    uint16_t feedback_angle_orig = (rxmsg->data8[0] << 8 |
                                    rxmsg->data8[1]);//set the present angle location by combining the data

#ifdef DEBUG_FEEDBACK_GIMBAL_ORIG_ANGLE
    if (motor_id == GIMBAL_MOTOR_YAW) {
        print("[GIMBAL_ORIG] Yaw = %d\n", (int)feedback_angle_orig);
    }
    else if (motor_id == GIMBAL_MOTOR_PIT) {
        print("[GIMBAL_ORIG] Pit = %d\n", (int)feedback_angle_orig);
    }
#endif

    //calculate the angle that the motor moves in degree, and we assume that the absolute value of the angle is smaller than 180 degrees
    float angle = ((float) feedback_angle_orig - (float) gimbal_fi_orig[motor_id]) * 360.0f / 8192.0f;
    //make sure that the angle is in [-180,180]
    if (angle < -180.0f)
        angle += 360.0f;
    if (angle > 180.0f)
        angle -= 360.0f;

    //update the actual angle
    motor[motor_id].actual_angle_last = motor[motor_id].actual_angle;//update the last gimbal angle to the angle before this change
    motor[motor_id].actual_angle = motor[motor_id].actual_angle + angle;//update the present angle
    if (motor[motor_id].actual_angle > 360.0f) {
        //if the actual_angle is greater than 360, then we can know that it has already turn a round in counter clockwise direction
        motor[motor_id].actual_angle -= 360.0f;//set the angle to be within [0,360]
        motor[motor_id].actual_angle_base_round++;//round count increases 1
    }
    if (motor[motor_id].actual_angle < 0) {
        //if the actual_angle is smaller than 0, then we can know that it has already turn a round in clockwise direction
        motor[motor_id].actual_angle += 360.0f;//set the angle to be within [0,360]
        motor[motor_id].actual_angle_base_round--;//round count decreases 1
    }
}

