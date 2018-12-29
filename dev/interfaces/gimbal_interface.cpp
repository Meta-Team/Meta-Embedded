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
    if (!yaw.enabled && !pitch.enabled) return true;

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
        txmsg.data8[0] = (uint8_t) (yaw.target_current >> 8); //upper byte
        txmsg.data8[1] = (uint8_t) yaw.target_current; // lower byte

    } else {
        txmsg.data8[0] = txmsg.data8[1] = 0;
    }


    if (pitch.enabled) {
#if GIMBAL_INTERFACE_ENABLE_CLIP
        ABS_LIMIT(pitch.target_current, GIMBAL_INTERFACE_MAX_CURRENT);
#endif
        txmsg.data8[2] = (uint8_t) (yaw.target_current >> 8); //upper byte
        txmsg.data8[3] = (uint8_t) yaw.target_current; // lower byte

    } else {
        txmsg.data8[2] = txmsg.data8[3] = 0;
    }

    txmsg.data8[4] =  txmsg.data8[5] = txmsg.data8[6] = txmsg.data8[7] = 0;

    can->send_msg(&txmsg);
}

bool GimbalInterface::process_motor_feedback(CANRxFrame *rxmsg) {

    motor_t* motor;
    if (rxmsg->SID == 0x205) motor = &yaw;
    else if (rxmsg->SID == 0x206) motor = &pitch;
    else
        return false;


    uint16_t feedback_angle_raw = (rxmsg->data8[0] << 8 |
                                    rxmsg->data8[1]); // set the present angle location by combining the data

    // calculate the angle that the motor moves in degree,
    // and we assume that the absolute value of the angle is smaller than 180 degrees
    float angle = (feedback_angle_raw - motor->front_angle_raw) * 360.0f / 8192.0f;
    //make sure that the angle is in [-180,180]
    if (angle < -180.0f)
        angle += 360.0f;
    if (angle > 180.0f)
        angle -= 360.0f;

    //update the actual angle
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