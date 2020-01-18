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
 *           Write target current to variables, then call send_chassis_currents to apply changes
 * @note This module is designed to process feedback automatically, but not to send current automatically, to avoid
 *       unintended chassis movements.
 */
class ChassisIF : public ChassisBase {

public:

    /**
     * Set CAN interface for receiving and sending
     * @param can_interface   Initialized CANInterface for chassis motors
     */
    static void init(CANInterface* can_interface);

    /** Structure for each motor */
    struct motor_feedback_t {

        motor_id_t id;

        uint16_t actual_angle_raw;
        int16_t actual_rpm_raw;
        int16_t actual_current_raw;
        uint8_t actual_temperature_raw;

        time_msecs_t last_update_time;

        float actual_velocity; // [degree/s]

    };

    /**
     * Feedback for each chassis motor
     */
    static motor_feedback_t feedback[];

    /**
     * Target current array in the order defined in motor_id_t
     */
    static int target_current[MOTOR_COUNT];

    /**
     * Send all target currents
     * @return Whether sending succeeded or not
     */
    static bool send_chassis_currents();

private:

    static CANInterface* can;

    /**
     * Callback function for CANInterface to process motor feedback
     * @param rxmsg
     */
    static void process_chassis_feedback(CANRxFrame const*rxmsg);

    friend CANInterface;

private:

    static float constexpr CHASSIS_MOTOR_DECELERATE_RATIO = 19.2f; // 3591/187 on the data sheet

};


#endif //META_INFANTRY_CHASSIS_INTERFACE_H

/** @} */
