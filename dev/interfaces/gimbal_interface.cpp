//
// Created by liuzikai on 2018-12-29.
// Zhu Kerui wrote code about processing gimbal feedback.
// Feng Chuhao wrote code about sending gimbal currents.
//

#include "gimbal_interface.h"
#include "common_macro.h"

GimbalInterface::motor_t GimbalInterface::yaw;
GimbalInterface::motor_t GimbalInterface::pitch;

CANInterface* GimbalInterface::can = nullptr;

bool GimbalInterface::send_gimbal_currents() {

    if (!can) return false;

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x1FF;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    if (yaw.enabled) {
#if GIMBAL_INTERFACE_ENABLE_CLIP
        ABS_LIMIT(yaw.target_current, GIMBAL_INTERFACE_MAX_CURRENT);
#endif
        /** NOTICE: target current is reversed. */
        txmsg.data8[0] = (uint8_t) (-yaw.target_current >> 8); //upper byte
        txmsg.data8[1] = (uint8_t) -yaw.target_current; // lower byte

    } else {
        txmsg.data8[0] = txmsg.data8[1] = 0;
    }


    if (pitch.enabled) {
#if GIMBAL_INTERFACE_ENABLE_CLIP
        ABS_LIMIT(pitch.target_current, GIMBAL_INTERFACE_MAX_CURRENT);
#endif
        /** NOTICE: target current is reversed. */
        txmsg.data8[2] = (uint8_t) (-pitch.target_current >> 8); //upper byte
        txmsg.data8[3] = (uint8_t) -pitch.target_current; // lower byte

    } else {
        txmsg.data8[2] = txmsg.data8[3] = 0;
    }

    txmsg.data8[4] =  txmsg.data8[5] = txmsg.data8[6] = txmsg.data8[7] = 0;

    can->send_msg(&txmsg);
    return true;
}

bool GimbalInterface::process_motor_feedback(CANRxFrame *rxmsg) {
/*
 * function logic description
 * first, get the absolute angle value from the motor, compared with the last absolute angle value, get the angle movement
 * add the angle movement with the relative angle, get the new relative angle, modify the relative angle and the base round value
 * divide the angle movement by the time break to get the angular velocity
 * */
    motor_t* motor;
    if (rxmsg->SID == 0x205) motor = &yaw;
    else if (rxmsg->SID == 0x206) motor = &pitch;
    else
        return false;

    // get the present absolute angle value by combining the data into a temporary variable
    uint16_t new_actual_angle_raw = (rxmsg->data8[0] << 8 | rxmsg->data8[1]);

    // check whether this new raw angle is valid
    if (new_actual_angle_raw > 8191){
        return false;
    }

    // calculate the angle movement in raw data
    // and we assume that the absolute value of the angle movement is smaller than 180 degrees(4096 of raw data)
    // currently motor->last_angle_raw holds the actual angle of last time
    int angle_movement = (int) new_actual_angle_raw - (int) motor->last_angle_raw;

    // update the last_angle_raw
    motor->last_angle_raw = new_actual_angle_raw;

    // make sure that the angle movement is in [-4096,4096] ([-180,180] in degree)
    if (angle_movement < -4096){
        angle_movement += 8192;
    }else if (angle_movement > 4096){
        angle_movement -= 8192;
    } else if(angle_movement == 4096 || angle_movement == -4096){
        if (motor->angular_velocity > 0){
            angle_movement = 4096;
        }else{
            angle_movement = -4096;
        }
    }

    // the key idea is to add the change of angle to actual angle.
    motor->actual_angle = motor->actual_angle + angle_movement * 360.0f / 8192;

    // modify the actual angle and update the round count when appropriate
    if (motor->actual_angle > 180.0f) {
        //if the actual_angle is greater than 360, then we can know that it has already turn a round in ***wise direction
        motor->actual_angle -= 360.0f;//set the angle to be within [0,360]
        motor->round_count++;//round count increases 1
    }
    if (motor->actual_angle < -180.0f) {
        // if the actual_angle is smaller than 0, then we can know that it has already turn a round in ***wise direction
        motor->actual_angle += 360.0f;//set the angle to be within [0,360]
        motor->round_count--;//round count decreases 1
    }

    // Sum angle movements for velocity_sample_interval times, and calculate the average.
    motor->sample_movement_sum += angle_movement;
    motor->sample_count++;
    if (motor->sample_count >= velocity_sample_interval) {
        // calculate the angular velocity
        time_msecs_t new_sample_time = TIME_I2MS(chibios_rt::System::getTime());
        motor->angular_velocity = motor->sample_movement_sum * 360.0f * 1000.0f/ 8192.0f / (float) (new_sample_time - motor->sample_time);
//        motor->angular_velocity = motor->sample_movement_sum * 360.0f / 8192.0f / (float) velocity_sample_interval / 0.001f;
        motor->sample_time = new_sample_time;

        motor->sample_movement_sum = 0;
        motor->sample_count = 0;
    }

    motor->actual_current = (int16_t)(rxmsg->data8[2] << 8 | rxmsg->data8[3]);

    return true;
}