//
// Created by liuzikai on 2019-01-12.
// Sending function is written by Feng Chuhao
// Receiving function is written by Qian Chen
//

/**
 * @file    chassis_interface.h
 * @brief   Interface to interact with low level driver of chassis, including processing chassis motor feedback and
 *          sending target currents.
 *
 * @addtogroup chassis
 * @{
 */

#ifndef META_INFANTRY_CHASSIS_INTERFACE_H
#define META_INFANTRY_CHASSIS_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"
#include "motor_interface.h"
#include "common_macro.h"

/**
 * Enable clip at the moment of sending current.
 * Only for safety. There is NO signal for clipping. Be sure to eliminate it if more current is needed.
 */
#define CHASSIS_INTERFACE_ENABLE_CLIP   FALSE

#if CHASSIS_INTERFACE_ENABLE_CLIP
#define CHASSIS_INTERFACE_MAX_CURRENT 5000  // mA
#endif

class ChassisBase {
public:
    // Start at the front right, goes in CCW
    enum motor_id_t {
        FR, // front right motor, 0
        FL, // front left motor, 1
        BL, // back left motor, 2
        BR, // back right motor, 3
        MOTOR_COUNT
    };
};

/**
 * @name ChassisIF
 * @note "IF" stands for "interface"
 * @brief Interface to process chassis motor feedback and send target current.
 * @pre Hardware is properly set. CAN id of each motor should be the same as motor_id_t.
 * @usage 1. Call init(CANInterface *). The interface should be properly initialized.
 *        2. Read feedback from variables.
 *           Write target current to variables, then call clip_chassis_current to apply changes
 * @note This module is designed to process feedback automatically, but not to send current automatically, to avoid
 *       unintended chassis movements.
 */
class ChassisIF : public ChassisBase, public MotorIFBase{

public:

    /**
     * Set CAN interface for receiving and sending
     * @param can1_            Initialized CANInterface for chassis motors
     * @param can2_            Initialized CANInterface for chassis motors
     * @param motor_can_config A group that contains ***ALL*** gimbal motor's info, include (can_channel, motor_can_id, motor_type)
     */
    static void init(CANInterface* can1_interface, CANInterface *can2_interface,
                     motor_can_config_t motor_can_config[MOTOR_COUNT]);

    /** Structure for each motor */

    /**
     * Feedback for each chassis motor
     */
    static CANInterface::motor_feedback_t *feedback[MOTOR_COUNT];

    /**
     * Target current array in the order defined in motor_id_t
     */
    static int *target_current[MOTOR_COUNT];

    /**
     * Clip chassis target currents (if enabled)
     */
    static void clip_chassis_current();

private:

    static CANInterface* can1_;
    static CANInterface* can2_;


};


#endif //META_INFANTRY_CHASSIS_INTERFACE_H

/** @} */
