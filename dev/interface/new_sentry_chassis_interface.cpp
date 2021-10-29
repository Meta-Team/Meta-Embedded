//
// Created by kerui on 2021/7/29.
//

#include "new_sentry_chassis_interface.h"

CANInterface *SChassisIF::can1_ = nullptr;
CANInterface *SChassisIF::can2_ = nullptr;

CANInterface::motor_feedback_t *SChassisIF::feedback[SChassisIF::MOTOR_COUNT];
int *SChassisIF::target_current[SChassisIF::MOTOR_COUNT];

void SChassisIF::init(CANInterface *can1_interface, CANInterface *can2_interface,
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