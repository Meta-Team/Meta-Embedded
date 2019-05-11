//
// Created by liuzikai on 2019-01-12.
// Sending function is written by Feng Chuhao
// Receiving function is written by Qian Cheng
//

#ifndef META_INFANTRY_CHASSIS_INTERFACE_H
#define META_INFANTRY_CHASSIS_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

/**
 * Enable clip at the moment of sending current.
 * Only for safety. There is NO signal for clipping. Be sure to eliminate it if more current is needed.
 */
#define CHASSIS_INTERFACE_ENABLE_CLIP                  TRUE

#if CHASSIS_INTERFACE_ENABLE_CLIP
#define CHASSIS_INTERFACE_MAX_CURRENT 5000  // mA
#endif

/**
 * @name ChassisInterface
 * @brief interface to process chassis motor feedback and send target current.
 * @pre hardware is properly set. CAN id of each motor should be the same as chassis_motor_id_t.
 * @usage 1. Call init(CANInterface *). The interface should be properly initialized.
 *        2. Control the data flow based on actual implementation
 * @note This module is designed to process feedback automatically, but not to send current automatically, to avoid
 *       unintended chassis movements.
 */
class ChassisInterface {

public:

    enum motor_id_t {  // goes in a counter-clockwise order
        FR, // front right motor, 0
        FL, // front left motor, 1
        BL, // back left motor, 2
        BR, // back right motor, 3
        MOTOR_COUNT
    };

    /**
     * @brief set CAN interface for receiving and sending
     * @param can_interface
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
     * @brief interface for each chassis motor
     */
    static motor_feedback_t feedback[];

    /**
     * @brief target current array in the order defined in motor_id_t
     */
    static int target_current[MOTOR_COUNT];

    /**
     * @brief send all target currents
     * @return
     */
    static bool send_chassis_currents();

private:

    static CANInterface* can;

    /**
     * @brief callback function for CANInterface to process motor feedback
     * @param rxmsg
     */
    static void process_chassis_feedback(CANRxFrame const*rxmsg);

    friend CANInterface;

private:

    /** Configurations **/

    static float constexpr chassis_motor_decelerate_ratio = 19.2f; // 3591/187 on the data sheet

};


#endif //META_INFANTRY_CHASSIS_INTERFACE_H
