//
// Created by admin on 2019/1/16.
//

#include "elevator_interface.h"
#include "common_macro.h"

int32_t ElevatorInterface::target_position_[2] = {0, 0};
ElevatorInterface::UnitInterface ElevatorInterface::wheels[4];
CANInterface *ElevatorInterface::can_ = nullptr;

bool ElevatorInterface::send_target_position_(int wheel_index) {

    CANTxFrame txFrame;

    // Set the PWM for the wheel.
    txFrame.IDE = CAN_IDE_STD;
    txFrame.RTR = CAN_RTR_DATA;
    txFrame.DLC = 0x08;
    txFrame.SID = 0x315 + 0x10 * (uint32_t) wheel_index;
    txFrame.data8[0] = (uint8_t) ((RMDS_DRIVER_PWM >> 8) & 0xFF);
    txFrame.data8[1] = (uint8_t) (RMDS_DRIVER_PWM & 0xFF);
    txFrame.data8[2] = txFrame.data8[3] = 0x55;
    txFrame.data8[4] = (uint8_t) ((target_position_[wheel_index / 2] >> 24) & 0xFF);
    txFrame.data8[5] = (uint8_t) ((target_position_[wheel_index / 2] >> 16) & 0xFF);
    txFrame.data8[6] = (uint8_t) ((target_position_[wheel_index / 2] >> 8) & 0xFF);
    txFrame.data8[7] = (uint8_t) (target_position_[wheel_index / 2] & 0xFF);

    can_->send_msg(&txFrame);

    return true;
}

bool ElevatorInterface::apply_front_position(float front_wheel_position_cm) {
//    if (front_wheel_position_cm > 0) return false;
    target_position_[0] = (int32_t) (front_wheel_position_cm * 40000);
    send_target_position_(FRONT_LEFT);
    send_target_position_(FRONT_RIGHT);
    wheels[0].is_actioning_ = wheels[1].is_actioning_ = true;
    LOG("[ElevatorInterface] Apply front pos %f", front_wheel_position_cm);
    return true;
}

bool ElevatorInterface::apply_rear_position(float rear_wheel_position_cm) {
//    if (rear_wheel_position_cm > 0) return false;
    target_position_[1] = (int32_t) (rear_wheel_position_cm * 40000);
    send_target_position_(REAR_LEFT);
    send_target_position_(REAR_RIGHT);
    wheels[2].is_actioning_ = wheels[3].is_actioning_ = true;
    LOG("[ElevatorInterface] Apply rear pos %f", rear_wheel_position_cm);
    return true;
}

void ElevatorInterface::process_feedback_(CANRxFrame const *rxmsg) {

    if (((rxmsg->SID >> 8) & 0xF) != RMDS_CAN_GROUP_ID) return;
    if ((rxmsg->SID & 0xF) != 0xB) return;

    unsigned index = ((rxmsg->SID >> 4) & 0xF) - 1; // wheel_id = can_id - 1

    if (index > WHEEL_COUNT) return;

    wheels[index].real_current = (rxmsg->data8[0] << 8) | rxmsg->data8[1];
    wheels[index].real_velocity = (rxmsg->data8[2] << 8) | rxmsg->data8[3];
    wheels[index].real_position =
            (rxmsg->data8[4] << 24) | (rxmsg->data8[5] << 16) | (rxmsg->data8[6] << 8) | rxmsg->data8[7];

    wheels[index].is_actioning_ = !ABS_IN_RANGE(
            wheels[index].real_position - target_position_[index / 2], RMDS_STABLE_RANGE);
}

void ElevatorInterface::init(CANInterface *can_interface) {

    /** Set and Register CAN interface **/
    can_ = can_interface;
    can_->register_callback(0x31B, 0x34B, process_feedback_);

    CANTxFrame txFrame;

    /** Initialize and set RMDS Module**/

    // Pre-fill info of txFrame
    txFrame.IDE = CAN_IDE_STD;
    txFrame.RTR = CAN_RTR_DATA;
    txFrame.DLC = 0x08;
    for (int data_index = 0; data_index < 8; data_index++) {
        txFrame.data8[data_index] = 0x55;
    }

    /* Step 1: Reset. Function ID = 0 */
    txFrame.SID = (RMDS_CAN_GROUP_ID << 8 | 0x00); // Use id 0 for broadcast
    can_->send_msg(&txFrame);
    chThdSleepMilliseconds(600); // waiting for the work to be done

    /* Step 2: Set the control mode to position mode (0x04). Function ID = 1 */
    txFrame.SID = (RMDS_CAN_GROUP_ID << 8 | 0x01); // Use id 0 for broadcast
    txFrame.data8[0] = 0x04;
    // Other data8 are still 0x55
    can_->send_msg(&txFrame);
    chThdSleepMilliseconds(500); // waiting for the work to be done

    /* Step 3: Choose the feedback mode for. Function ID = A */
    txFrame.SID = (RMDS_CAN_GROUP_ID << 8 | 0x0A); // Use id 0 for broadcast
    txFrame.data8[0] = RMDS_FEEDBACK_INTERVAL; // turn on feedback of instant current, velocity and position
    txFrame.data8[1] = 0x00;              // turn off feedback of voltages of motor driving pins and PWM
    // Other data8 are still 0x55
    can_->send_msg(&txFrame);

    /* Step 4: Set the PWM and target position = 0 for the four wheels. Function ID = 5 */
    txFrame.SID = (RMDS_CAN_GROUP_ID << 8 | 0x05); // Use id 0 for broadcast
    txFrame.data8[0] = (uint8_t) ((RMDS_DRIVER_PWM >> 8) & 0xFF);
    txFrame.data8[1] = (uint8_t) (RMDS_DRIVER_PWM & 0xFF);
    txFrame.data8[4] = txFrame.data8[5] = txFrame.data8[6] = txFrame.data8[7] = 0;
    can_->send_msg(&txFrame);
}

bool ElevatorInterface::UnitInterface::is_in_action() {
    return is_actioning_;
}