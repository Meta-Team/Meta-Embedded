//
// Created by liuzikai on 2019-01-12.
//

/**
 * @file    chassis_interface.cpp
 * @brief   Interface to interact with low level driver of chassis, including processing chassis motor feedback and
 *          sending target currents.
 *
 * @addtogroup chassis
 * @{
 */

#include "chassis_interface.h"

CANInterface *ChassisIF::can1_ = nullptr;
CANInterface *ChassisIF::can2_ = nullptr;

CANInterface::motor_feedback_t *ChassisIF::feedback[ChassisIF::MOTOR_COUNT];
int *ChassisIF::target_current[ChassisIF::MOTOR_COUNT];

void ChassisIF::clip_chassis_current() {

#if CHASSIS_INTERFACE_ENABLE_CLIP
        ABS_CROP(*target_current[i], CHASSIS_INTERFACE_MAX_CURRENT);
#endif

}

void ChassisIF::init(CANInterface *can1_interface, CANInterface *can2_interface,
                                motor_can_config_t motor_can_config[MOTOR_COUNT]) {
    can1_ = can1_interface;
    can2_ = can2_interface;
    for (int i = 0; i < MOTOR_COUNT; i++) {

        if (motor_can_config[i].motor_can_channel == can_channel_1) {

            // Link the feedback and target_current to can1's feedback
            feedback[i] = can1_->register_feedback_address(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type, motor_can_config[i].deceleration_ratio);
            if (feedback[i] == nullptr) while(true) LOG_ERR("ChassisIF: failed to register motor %d feedback", i);
            target_current[i] = can1_->register_target_current_address(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type);
            if (target_current[i] == nullptr) while(true) LOG_ERR("ChassisIF: failed to register motor %d target_current", i);
            *target_current[i] = 0;

        } else if (motor_can_config[i].motor_can_channel == can_channel_2) {

            // Link the feedback and target_current to can2's feedback
            feedback[i] = can2_->register_feedback_address(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type, motor_can_config[i].deceleration_ratio);
            if (feedback[i] == nullptr) while(true) LOG_ERR("ChassisIF: failed to register motor %d feedback", i);
            target_current[i] = can2_->register_target_current_address(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type);
            if (target_current[i] == nullptr) while(true) LOG_ERR("ChassisIF: failed to register motor %d target_current", i);
            *target_current[i] = 0;

        }
    }
}

/** @} */