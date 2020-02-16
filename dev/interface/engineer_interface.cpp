//
// Created by zzb on 2020/1/18.
//

#include "engineer_interface.h"
#include "can_interface.h"
CANInterface* EngineerInterface::can1_ = nullptr;
CANInterface* EngineerInterface::can2_ = nullptr;
CANInterface::motor_feedback_t* EngineerInterface::feedback[4];
int* EngineerInterface::target_current[4];


int EngineerInterface::door_state = 0;  //close the door

void EngineerInterface::init( CANInterface *can1_interface,                      CANInterface *can2_interface,
                              motor_can_config_t motor_can_config[MOTOR_COUNT]){
    EngineerInterface::can1_ = can1_interface;
    EngineerInterface::can2_ = can2_interface;

    for (int i = 0; i < MOTOR_COUNT; i++) {

        // Check can config validation, id > 7, none motor, none can channel would set the feedback to a void zone.
        if(motor_can_config[i].motor_can_id > 7 ||
           motor_can_config[i].motor_type == CANInterface::NONE_MOTOR ||
           motor_can_config[i].motor_can_channel == none_can_channel) {

            // Make the pointer to the "deadzone", which could be easily judged that the motor is invalid.
            feedback[i] = can1_->get_feedback_address(CANInterface::MAXIMUM_MOTOR_COUNT);
            target_current[i] = can1_->get_target_current_address(CANInterface::MAXIMUM_MOTOR_COUNT);
            continue;
        }

        // Link feedback and target current address to can interface.
        if (motor_can_config[i].motor_can_channel == can_channel_1) {

            // Link the feedback and target_current to can1's feedback
            can1_->set_motor_type(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type);
            feedback[i] = can1_->get_feedback_address(motor_can_config[i].motor_can_id);
            target_current[i] = can1_->get_target_current_address(motor_can_config[i].motor_can_id);
            *target_current[i] = 0;

        } else if (motor_can_config[i].motor_can_channel == can_channel_2) {

            // Link the feedback and target_current to can2's feedback
            can2_->set_motor_type(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type);
            feedback[i] = can2_->get_feedback_address(motor_can_config[i].motor_can_id);
            target_current[i] = can2_->get_target_current_address(motor_can_config[i].motor_can_id);
            *target_current[i] = 0;

        }
    }

    // We don't set front angle because there's a deceleration ratio of M2006 and RM3508 motors. The front angle
    // indicate the angle of rotor but not the angle of rotating axis.

    chThdSleep(TIME_MS2I(500));
}

void EngineerInterface::change_door(){

}

void EngineerInterface::enable_engr_current_clip() {

#if ENGR_INTERFACE_ENABLE_CLIP
    ABS_CROP(*target_current[RESCUE_LEFT], ENGR_RESCUE_INTERFACE_MAX_CURRENT);
#endif

    // Fill the current of Pitch
#if ENGR_INTERFACE_ENABLE_CLIP
    ABS_CROP(*target_current[RESCUE_RIGHT], ENGR_RESCUE_INTERFACE_MAX_CURRENT);
#endif

    // Fill the current of bullet loader
#if ENGR_INTERFACE_ENABLE_CLIP
    ABS_CROP(*target_current[ROBOTIC_LEFT], ENGR_ROBOTIC_INTERFACE_MAX_CURRENT);
#endif

    // Fill the current of plate
#if ENGR_INTERFACE_ENABLE_CLIP
    ABS_CROP(*target_current[ROBOTIC_RIGHT], ENGR_ROBOTIC_INTERFACE_MAX_CURRENT);
#endif

}
