//
// Created by admin on 2019/1/16.
//

#include "elevator_interface.h"

int32_t ElevatorInterface::target_position[2];
ElevatorInterface::UnitInterface ElevatorInterface::elevator_wheels[4] = {GPIOE_PIN6, GPIOE_PIN12, GPIOE_PIN5, GPIOE_PIN4};
CANInterface *ElevatorInterface::can = nullptr;


bool ElevatorInterface::send_target_position() {

    for (int wheel_index = 0; wheel_index < WHEEL_COUNT; wheel_index++) {
        CANTxFrame txFrame;

        // Set the PWM for the wheel.
        txFrame.IDE = CAN_IDE_STD;
        txFrame.RTR = CAN_RTR_DATA;
        txFrame.DLC = 0x08;
        txFrame.SID = 0x315 + 0x10 * (uint32_t) wheel_index;
        txFrame.data8[0] = (uint8_t) ((driver_pwm >> 8) & 0xFF);
        txFrame.data8[1] = (uint8_t) (driver_pwm & 0xFF);
        txFrame.data8[2] = txFrame.data8[3] = 0x55;
        txFrame.data8[4] = (uint8_t) ((target_position[wheel_index / 2] >> 24) & 0xFF);
        txFrame.data8[5] = (uint8_t) ((target_position[wheel_index / 2] >> 16) & 0xFF);
        txFrame.data8[6] = (uint8_t) ((target_position[wheel_index / 2] >> 8) & 0xFF);
        txFrame.data8[7] = (uint8_t) (target_position[wheel_index / 2] & 0xFF);

        can->send_msg(&txFrame);

    }

    return true;
}

void ElevatorInterface::set_target_position(float front_wheel_position_cm, float rear_wheel_position_cm) {
    target_position[0] = (int32_t) (front_wheel_position_cm * 40000);
    target_position[1] = (int32_t) (rear_wheel_position_cm * 40000);
}

bool ElevatorInterface::process_feedback(CANRxFrame *rxmsg) {
    if (((rxmsg->SID >> 8) & 0xF) != can_group_id) return false;
    if ((rxmsg->SID & 0xF) != 0xB) return false;

    unsigned int index = ((rxmsg->SID >> 4) & 0xF) - 1; // wheel_id = can_id - 1

    if (index > WHEEL_COUNT) return false;

    elevator_wheels[index].real_current = (rxmsg->data8[0] << 8) | rxmsg->data8[1];
    elevator_wheels[index].real_velocity = (rxmsg->data8[2] << 8) | rxmsg->data8[3];
    elevator_wheels[index].real_position =
            (rxmsg->data8[4] << 24) | (rxmsg->data8[5] << 16) | (rxmsg->data8[6] << 8) | rxmsg->data8[7];

    return true;
}

void ElevatorInterface::init(CANInterface *can_interface) {

    can = can_interface;

    CANTxFrame txFrame;

    // Pre-fill info of txFrame
    txFrame.IDE = CAN_IDE_STD;
    txFrame.RTR = CAN_RTR_DATA;
    txFrame.DLC = 0x08;
    for (int data_index = 0; data_index < 8; data_index++) {
        txFrame.data8[data_index] = 0x55;
    }

    /* Step 1: Reset. Function ID = 0 */
    txFrame.SID = (can_group_id << 8 | 0x00); // Use id 0 for broadcast
    can->send_msg(&txFrame);
    chThdSleepMilliseconds(600); // waiting for the work to be done

    /* Step 2: Set the control mode to position mode (0x04). Function ID = 1 */
    txFrame.SID = (can_group_id << 8 | 0x01); // Use id 0 for broadcast
    txFrame.data8[0] = 0x04;
    // Other data8 are still 0x55
    can->send_msg(&txFrame);
    chThdSleepMilliseconds(500); // waiting for the work to be done

    /* Step 3: Choose the feedback mode for. Function ID = A */
    txFrame.SID = (can_group_id << 8 | 0x0A); // Use id 0 for broadcast
    txFrame.data8[0] = feedback_interval; // turn on feedback of instant current, velocity and position
    txFrame.data8[1] = 0x00;              // turn off feedback of voltages of motor driving pins and PWM
    // Other data8 are still 0x55
    can->send_msg(&txFrame);


    /* Step 4: Set the PWM for the four wheels. Function ID = 5 */
    txFrame.SID = (can_group_id << 8 | 0x05); // Use id 0 for broadcast
    for (int wheel_index = 0; wheel_index < 4; wheel_index++) {
        txFrame.data8[0] = (uint8_t) ((driver_pwm >> 8) & 0xFF);
        txFrame.data8[1] = (uint8_t) (driver_pwm & 0xFF);
    }
    // Other data8 are still 0x55
    can->send_msg(&txFrame);
}

bool ElevatorInterface::UnitInterface::get_safety_button_status() {
    return (palReadPad(ELEVATOR_INTERFACE_SAFETY_BUTTON_PAD, safety_button_pin) == PAL_HIGH);
}