//
// Created by admin on 2019/1/16.
//

#include "elevator_interface.h"

int32_t ElevatorInterface::target_position[2];
CANTxFrame ElevatorInterface::txFrames[4];
ElevatorInterface::elevator_wheel_t ElevatorInterface::elevator_wheels[4];
CANInterface* ElevatorInterface::can = nullptr;
uint8_t ElevatorInterface::feedback_time;


bool ElevatorInterface::send_message() {

    uint16_t ButtonsStatus[4][2];// Use it to detect if the elevators are on the bottom.

    ButtonsStatus[0][0]=palSetPad(GPIOF,GPIOF_PINUP);
    ButtonsStatus[0][1]=palSetPad(GPIOF,GPIOF_PINDOWN); // The FRONT_LEFT_BUTTONS.
    ButtonsStatus[1][0]=palSetPad(GPIOF,GPIOF_PINUP);
    ButtonsStatus[1][1]=palSetPad(GPIOF,GPIOF_PINDOWN); // The FRONT_RIGHT_BUTTONS.
    ButtonsStatus[2][0]=palSetPad(GPIOF,GPIOF_PINUP);
    ButtonsStatus[2][1]=palSetPad(GPIOF,GPIOF_PINDOWN); // The REAR_LEFT_BUTTONS.
    ButtonsStatus[3][0]=palSetPad(GPIOF,GPIOF_PINUP);
    ButtonsStatus[3][1]=palSetPad(GPIOF,GPIOF_PINDOWN); // The REAR_RIGHT_BUTTONS.

    for(uint32_t wheel_index = 0; wheel_index < 4; wheel_index++){
        for(int upordown = 0; upordown < 2; upordown++){ // Up 0; down 1.
            if( (ButtonsStatus[wheel_index][upordown]==0) && ((target_position-elevator_wheels[wheel_index].real_position) * (upordown * 2 - 1) < 0 )) { // When reaches to the edge, but still tries going out of the edge.
                if (upordown == 1){ // When wheel touches the bottom.
                    elevator_wheels[wheel_index].real_position = 0;
                }

                // Reset the wheel respectively.

                txFrames[wheel_index].SID = 0x310 + 0x10 * wheel_index;
                txFrames[wheel_index].IDE = CAN_IDE_STD;
                txFrames[wheel_index].RTR = CAN_RTR_DATA;
                txFrames[wheel_index].DLC = 0x08;
                for(int data_index = 0; data_index < 8; data_index++ ){
                    txFrames[wheel_index].data8[data_index] = 0x55;
                }
                can->send_msg(&txFrames[wheel_index]);

                chThdSleepMilliseconds(600); // Wait for the work to be done.

                // Choose the Control mode for the wheel.
                txFrames[wheel_index].SID = 0x311 + 0x10 * wheel_index;
                txFrames[wheel_index].data8[0] = 0x04;
                can->send_msg(&txFrames[wheel_index]);

                chThdSleepMilliseconds(500); // Wait for the work to be done.

                // Choose the FeedBack mode for the wheel.
                txFrames[wheel_index].SID = 0x31A + 0x10 * wheel_index;

                feedback_time = 100;
                txFrames[wheel_index].data8[0] = feedback_time;
                txFrames[wheel_index].data8[1] = 0x00;
                can->send_msg(&txFrames[wheel_index]);

                // Set the PWM for the wheel.
                txFrames[wheel_index].SID = 0x315 + 0x10 * wheel_index;
                txFrames[wheel_index].data8[0] = (unsigned char) ((PWM >> 8) & 0Xff);
                txFrames[wheel_index].data8[1] = (unsigned char) (PWM & 0Xff);
            }
            else{ // When nothing goes wrong.
                int dd = wheel_index/2; // Replace the i.

                txFrames[wheel_index].data8[4] = (unsigned char) ((target_position[dd] >> 24) &
                                                                    0Xff);
                txFrames[wheel_index].data8[5] = (unsigned char) ((target_position[dd] >> 16) &
                                                                    0Xff);
                txFrames[wheel_index].data8[6] = (unsigned char) ((target_position[dd] >> 8 ) &
                                                                    0Xff);
                txFrames[wheel_index].data8[7] = (unsigned char) (target_position[dd] & 0Xff);

                can->send_msg(&txFrames[wheel_index]);
            }
        }
    }

    return true;
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
    elevator_wheels[index].real_current = (rxmsg->data8[0]<<8)|rxmsg->data8[1];
    elevator_wheels[index].real_velocity = (rxmsg->data8[2]<<8)|rxmsg->data8[3];
    elevator_wheels[index].real_position = (rxmsg->data8[4]<<24)|(rxmsg->data8[5]<<16)|(rxmsg->data8[6]<<8)|rxmsg->data8[7];
    return true;
}

void ElevatorInterface::start(CANInterface* can_interface) {

    can = can_interface;
    // First enter the current mode, let wheels goes to the bottom (Stop respectively), when all reach to the bottom, restart the for wheels and enter the position mode.
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
        txFrames[wheel_index].data8[0] = 0x04;
        can->send_msg(&txFrames[wheel_index]);
    }

    // Waiting for the work to be done
    chThdSleepMilliseconds(500);

    // Choose the feedback mode for the four wheels
    txFrames[0].SID = COMMAND_A_FRONT_LEFT;
    txFrames[1].SID = COMMAND_A_FRONT_RIGHT;
    txFrames[2].SID = COMMAND_A_REAR_LEFT;
    txFrames[3].SID = COMMAND_A_REAR_RIGHT;

    feedback_time = 100;

    for(int wheel_index = 0; wheel_index < 4; wheel_index++){
        txFrames[wheel_index].data8[0] = feedback_time;
        txFrames[wheel_index].data8[1] = 0x00;
        can->send_msg(&txFrames[wheel_index]);
    }

    // Set the PWM for the four wheels
    txFrames[0].SID = COMMAND_5_FRONT_LEFT;
    txFrames[1].SID = COMMAND_5_FRONT_RIGHT;
    txFrames[2].SID = COMMAND_5_REAR_LEFT;
    txFrames[3].SID = COMMAND_5_REAR_RIGHT;
    for(int wheel_index = 0; wheel_index < 4; wheel_index++){
        txFrames[wheel_index].data8[0] = (unsigned char) ((PWM >> 8) & 0Xff);
        txFrames[wheel_index].data8[1] = (unsigned char) (PWM & 0Xff);
    }

    // Restart four wheels respectively. Use while to let the procedure repeat again and again.
    // Only when for wheels all touch to the bottom, the while loop over.

    uint16_t ButtonsStatus[4][2];// Use it to detect if the elevators are on the bottom.

    ButtonsStatus[0]=palSetPad(GPIOF,GPIOF_PINDOWN); // The FRONT_LEFT_BUTTONS.
    ButtonsStatus[1]=palSetPad(GPIOF,GPIOF_PINDOWN); // The FRONT_RIGHT_BUTTONS.
    ButtonsStatus[2]=palSetPad(GPIOF,GPIOF_PINDOWN); // The REAR_LEFT_BUTTONS.
    ButtonsStatus[3]=palSetPad(GPIOF,GPIOF_PINDOWN); // The REAR_RIGHT_BUTTONS.
    while (ButtonsStatus[0]+ButtonsStatus[1]+ButtonsStatus[2]+ButtonsStatus[3] != 0) {
        for (uint32_t wheel_index = 0; wheel_index < 4; wheel_index++) {
            if ((ButtonsStatus[wheel_index]== 0){ // When reaches to the bottom.
                elevator_wheels[wheel_index].real_position = 0;

                // Reset the wheel respectively.

                txFrames[wheel_index].SID = 0x310 + 0x10 * wheel_index;
                txFrames[wheel_index].IDE = CAN_IDE_STD;
                txFrames[wheel_index].RTR = CAN_RTR_DATA;
                txFrames[wheel_index].DLC = 0x08;
                for (int data_index = 0; data_index < 8; data_index++) {
                    txFrames[wheel_index].data8[data_index] = 0x55;
                }
                can->send_msg(&txFrames[wheel_index]);

                chThdSleepMilliseconds(600); // Wait for the work to be done.

                // Choose the Control mode for the wheel.
                txFrames[wheel_index].SID = 0x311 + 0x10 * wheel_index;
                txFrames[wheel_index].data8[0] = 0x04;
                can->send_msg(&txFrames[wheel_index]);

                chThdSleepMilliseconds(500); // Wait for the work to be done.

                // Choose the FeedBack mode for the wheel.
                txFrames[wheel_index].SID = 0x31A + 0x10 * wheel_index;

                feedback_time = 100;
                txFrames[wheel_index].data8[0] = feedback_time;
                txFrames[wheel_index].data8[1] = 0x00;
                can->send_msg(&txFrames[wheel_index]);

                // Set the PWM for the wheel.
                txFrames[wheel_index].SID = 0x315 + 0x10 * wheel_index;
                txFrames[wheel_index].data8[0] = (unsigned char) ((PWM >> 8) & 0Xff);
                txFrames[wheel_index].data8[1] = (unsigned char) (PWM & 0Xff);
            }
            else { // When nothing goes wrong. Let the elevator goes down.
                int dd = wheel_index / 2; // Replace the original i.

                set_position(-40000,-40000); // Goes down crazily.

                txFrames[wheel_index].data8[4] = (unsigned char) ((target_position[dd] >> 24) &
                                                                      0Xff);
                txFrames[wheel_index].data8[5] = (unsigned char) ((target_position[dd] >> 16) &
                                                                      0Xff);
                txFrames[wheel_index].data8[6] = (unsigned char) ((target_position[dd] >> 8) &
                                                                      0Xff);
                txFrames[wheel_index].data8[7] = (unsigned char) (target_position[dd] & 0Xff);

                can->send_msg(&txFrames[wheel_index]);
            }
        }
    }
}
