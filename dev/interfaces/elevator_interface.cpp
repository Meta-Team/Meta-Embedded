//
// Created by admin on 2019/1/16.
//

#include "elevator_interface.h"

bool ElevatorInterface::send_message() {
    for(int i = 0; i < 2; i++) {
        txFrames[i * 2].data8[4] = txFrames[i * 2 + 1].data8[4] = (unsigned char) ((target_position[i] >> 24) &
                                                                                   0Xff);
        txFrames[i * 2].data8[5] = txFrames[i * 2 + 1].data8[5] = (unsigned char) ((target_position[i] >> 16) &
                                                                                   0Xff);
        txFrames[i * 2].data8[6] = txFrames[i * 2 + 1].data8[6] = (unsigned char) ((target_position[i] >> 8) &
                                                                                   0Xff);
        txFrames[i * 2].data8[7] = txFrames[i * 2 + 1].data8[7] = (unsigned char) (target_position[i] & 0Xff);
        can->send_msg(&txFrames[i * 2]);
        can->send_msg(&txFrames[i * 2 + 1]);
    }
}

void ElevatorInterface::set_position(int32_t front_wheel_position, int32_t rear_wheel_position) {
    target_position[0] = front_wheel_position;
    target_position[1] = rear_wheel_position;
}

bool ElevatorInterface::get_feedback(CANRxFrame *rxmsg) {
    int index;
    switch (rxmsg->SID){
        case RX_FRONT_LEFT:
            index = 0;
            break;
        case RX_FRONT_RIGHT:
            index = 1;
            break;
        case RX_REAR_LEFT:
            index = 2;
            break;
        case RX_REAR_RIGHT:
            index = 3;
            break;
        default:
            return false;
    }
    elevator_wheels[index].real_current = (rxmsg.data8[0]<<8)|rxmsg.data8[1];
    elevator_wheels[index].real_velocity = (rxmsg.data8[2]<<8)|rxmsg.data8[3];
    elevator_wheels[index].real_position = (rxmsg.data8[4]<<24)|(rxmsg.data8[5]<<16)|(rxmsg.data8[6]<<8)|rxmsg.data8[7];
    return true;
}

void ElevatorInterface::start(CANInterface* can_interface) {
    can = can_interface;

    // Reset the four wheels
    txFrames[0].SID = COMMAND_0_FRONT_LEFT;
    txFrames[1].SID = COMMAND_0_FRONT_RIGHT;
    txFrames[2].SID = COMMAND_0_REAR_LEFT;
    txFrames[3].SID = COMMAND_0_REAR_RIGHT;

    for(int wheel_index = 0; wheel_index < 4; wheel_index++){
        txFrames[wheel_index].IDE = CAN_IDE_STD;
        txFrames[wheel_index].RTR = CAN_RTR_DATA;
        txFrames[wheel_index].DLC = 0x08;
        for(int data_index = 0; data_index < 8; data_index++){
            txFrames[wheel_index].data8[data_index] = 0x55;
        }
        can->send_msg(&txFrames[wheel_index]);
    }

    // Waiting for the work to be done
    chThdSleepMilliseconds(600);

    // Choose the control mode for the four wheels
    txFrames[0].SID = COMMAND_1_FRONT_LEFT;
    txFrames[1].SID = COMMAND_1_FRONT_RIGHT;
    txFrames[2].SID = COMMAND_1_REAR_LEFT;
    txFrames[3].SID = COMMAND_1_REAR_RIGHT;

    for(int wheel_index = 0; wheel_index < 4; wheel_index++){
        txFrames[wheel_index].data8[0] = 0x05;
        can->send_msg(&txFrames[wheel_index]);
    }

    // Waiting for the work to be done
    chThdSleepMilliseconds(100);

    // Set the PWM for the four wheels
    txFrames[0].SID = COMMAND_5_FRONT_LEFT;
    txFrames[1].SID = COMMAND_5_FRONT_RIGHT;
    txFrames[2].SID = COMMAND_5_REAR_LEFT;
    txFrames[3].SID = COMMAND_5_REAR_RIGHT;

    for(int wheel_index = 0; wheel_index < 4; wheel_index++){
        txFrames[wheel_index].data8[0] = (unsigned char) ((PWM >> 8) & 0Xff);
        txFrames[wheel_index].data8[1] = (unsigned char) (PWM & 0Xff);
    }
}