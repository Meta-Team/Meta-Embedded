//
// Created by Kerui Zhu on 7/9/2019.
//

#ifndef META_INFANTRY_ENGINEER_ELEVATOR_INTERFACE_H
#define META_INFANTRY_ENGINEER_ELEVATOR_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"
#include "common_macro.h"
#include "../vehicle/engineer/vehicle_engineer.h"

/**
 * @name EngineerElevatorIF
 * @brief interface to process elevator motor feedback and send target current.
 * @pre hardware is properly set. CAN id of each motor should be the same as motor_id_t.
 * @usage 1. Call init(CANInterface *). The interface should be properly initialized.
 *        2. Control the data flow based on actual implementation
 * @note This module is designed to process feedback automatically, but not to send current automatically, to avoid
 *       unintended elevator movements.
 */

class EngineerElevatorIF {

public:

    enum motor_id_t {
        R, // right motor, 0
        L, // left motor, 1
        MOTOR_COUNT
    };


    /** Structure for each motor */
    class elevator_motor_t {

    public:

        float present_angle; // [degree]
        float actual_velocity; // [degree/s]
        int16_t actual_current;
        time_msecs_t last_update_time;
        int16_t target_current;

        /** @brief set current actual angle as 0 degree */
        void clear_accumulate_angle();

    private:
        uint16_t actual_angle_raw;
        int16_t actual_rpm_raw;
        friend EngineerElevatorIF;
    };


    struct aided_motor_t {
        float actual_velocity; // [degree/s]
        time_msecs_t last_update_time;
        int16_t target_current;
    };


    static elevator_motor_t elevatorMotor[];
    static aided_motor_t aidedMotor[];

    /**
     * @brief set CAN interface for receiving and sending
     * @param can_interface
     */
    static void init(CANInterface* can_interface);

    static float get_current_height();

    static bool send_currents();

private:

    static CANInterface* can;

    /**
     * @brief callback function for CANInterface to process motor feedback
     * @param rxmsg
     */
    static void process_feedback(CANRxFrame const*rxmsg);

    friend CANInterface;

private:

    /** Configurations **/

    static float constexpr chassis_motor_decelerate_ratio = 19.2f; // 3591/187 on the data sheet
};

#endif //META_INFANTRY_ENGINEER_ELEVATOR_INTERFACE_H
