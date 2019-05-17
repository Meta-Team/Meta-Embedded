//
// Created by Zhu Kerui on 2019/1/16.
//

#ifndef META_INFANTRY_RMDS108_INTERFACE_H
#define META_INFANTRY_RMDS108_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

#include "common_macro.h"

/**
 * @name ElevatorInterface
 * @brief interface to process elevator motor feedback and send target current.
 * @pre hardware is properly set. CAN id of each motor should be the same as motor_id_t.
 * @usage 1. Call init(CANInterface *). The interface should be properly initialized.
 *        2. Control the data flow based on actual implementation
 * @note This module is designed to process feedback automatically, but not to send current automatically, to avoid
 *       unintended elevator movements.
 */
class ElevatorInterface {

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

        int accmulate_angle;      // [degree]

        float actual_velocity; // [degree/s]
        bool in_action;

        /**
         * @brief set current actual angle as 0 degree
         */
        void clear_accmulate_angle();

    };

    /**
     * @brief interface for each chassis motor
     */
    static motor_feedback_t feedback[];

    /**
     * @brief target current array in the order defined in motor_id_t
     */
    static int target_current[MOTOR_COUNT];
    static int target_angle[MOTOR_COUNT];

    /**
     * @brief send all target currents
     * @return
     */
    static bool send_elevator_currents();

private:

    static CANInterface* can;

    /**
     * @brief callback function for CANInterface to process motor feedback
     * @param rxmsg
     */
    static void process_elevator_feedback(CANRxFrame const*rxmsg);

    static constexpr int STABLE_RANGE = 0;

    friend CANInterface;

private:

    /** Configurations **/

    static float constexpr chassis_motor_decelerate_ratio = 19.2f; // 3591/187 on the data sheet

};

#endif //META_INFANTRY_RMDS108_INTERFACE_H
