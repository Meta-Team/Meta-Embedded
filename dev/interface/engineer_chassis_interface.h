//
// Created by Kerui Zhu on 7/9/2019.
//

#ifndef META_INFANTRY_ENGINEER_CHASSIS_INTERFACE_H
#define META_INFANTRY_ENGINEER_CHASSIS_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"
#include "common_macro.h"

/**
 * Enable clip at the moment of sending current.
 * Only for safety. There is NO signal for clipping. Be sure to eliminate it if more current is needed.
 */
#define CHASSIS_INTERFACE_ENABLE_CLIP                  FALSE

#if CHASSIS_INTERFACE_ENABLE_CLIP
#define CHASSIS_INTERFACE_MAX_CURRENT 5000  // mA
#endif

#define ENGINEER_CHASSIS_MOTOR_COUNT 4

/**
 * @name EngineerChassisIF
 * @brief interface to process Engineer chassis motor feedback and send target current.
 * @pre hardware is properly set. CAN id of each motor should be the same as motor_id_t.
 * @usage 1. Call init(CANInterface *). The interface should be properly initialized.
 *        2. Control the data flow based on actual implementation
 * @note This module is designed to process feedback automatically, but not to send current automatically, to avoid
 *       unintended chassis movements.
 */

class EngineerChassisIF {

public:

    enum motor_id_t {  // goes in a counter-clockwise order
        FR, // front right motor, 0
        FL, // front left motor, 1
        BL, // back left motor, 2
        BR, // back right motor, 3
    };

    /** Structure for each motor */
    struct motor_t {

        uint16_t actual_angle_raw;
        int16_t actual_rpm_raw;
        int16_t actual_current_raw;
        uint8_t actual_temperature_raw;

        time_msecs_t last_update_time;

        float actual_velocity; // [degree/s]
        uint16_t target_current;
    };

    static motor_t motors[];

    static void init(CANInterface* can_interface);

    static bool send_currents();

private:

    static CANInterface* can;

    static void process_feedback(CANRxFrame const*rxmsg);

    friend CANInterface;

    /** Constant Parameters **/

    static float constexpr chassis_motor_decelerate_ratio = 19.2f; // 3591/187 on the data sheet

};

#endif //META_INFANTRY_ENGINEER_CHASSIS_INTERFACE_H
