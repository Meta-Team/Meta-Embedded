//
// Created by admin on 2019/1/16.
//

#include "elevator_interface.h"

bool ElevatorInterface::send_message() {
    for(int i = 0; i < 2; i++) {
        if (enabled) {
            txFrames[i * 2].data8[0] = txFrames[i * 2 + 1].data8[0] = (unsigned char) ((PWM >> 8) & 0Xff);
            txFrames[i * 2].data8[1] = txFrames[i * 2 + 1].data8[1] = (unsigned char) ((PWM) & 0Xff);
            txFrames[i * 2].data8[2] = txFrames[i * 2].data8[3] = txFrames[i * 2 + 1].data8[2] = txFrames[i * 2 +
                                                                                                          1].data8[3] = 0x55;
            txFrames[i * 2].data8[4] = txFrames[i * 2 + 1].data8[4] = (unsigned char) ((target_position[i] >> 24) &
                                                                                       0Xff);
            txFrames[i * 2].data8[5] = txFrames[i * 2 + 1].data8[5] = (unsigned char) ((target_position[i] >> 16) &
                                                                                       0Xff);
            txFrames[i * 2].data8[6] = txFrames[i * 2 + 1].data8[6] = (unsigned char) ((target_position[i] >> 8) &
                                                                                       0Xff);
            txFrames[i * 2].data8[7] = txFrames[i * 2 + 1].data8[7] = (unsigned char) (target_position[i] & 0Xff);
        } else {
            /*do something when enabled is false*/
        }
    }
    can->send_msg(&txFrames[0]);
    can->send_msg(&txFrames[1]);
    can->send_msg(&txFrames[2]);
    can->send_msg(&txFrames[3]);
}

void ElevatorInterface::set_position(int32_t front_wheel_position, int32_t rear_wheel_position) {
    target_position[0] = front_wheel_position;
    target_position[1] = rear_wheel_position;
}

bool ElevatorInterface::get_feedback(CANRxFrame *rxmsg) {
    int id;
    switch (rxmsg->SID){
        case FRONT_LEFT:
            id = 0;
            break;
        case FRONT_RIGHT:
            id = 1;
            break;
        case REAR_LEFT:
            id = 2;
            break;
        case REAR_RIGHT:
            id = 3;
            break;
        default:
            id = 0;
    }
    elevator_wheels[id].real_current = (rxmsg.data8[0]<<8)|rxmsg.data8[1];
    elevator_wheels[id].real_velocity = (rxmsg.data8[2]<<8)|rxmsg.data8[3];
    elevator_wheels[id].real_position = (rxmsg.data8[4]<<24)|(rxmsg.data8[5]<<16)|(rxmsg.data8[6]<<8)|rxmsg.data8[7];
    return true;
}