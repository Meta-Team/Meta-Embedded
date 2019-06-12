//
// Created by liuzikai on 2019-04-12.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_H
#define META_INFANTRY_SENTRY_CHASSIS_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

/**
 * Enable clip at the moment of sending current.
 * Only for safety. There is NO signal for clipping. Be sure to eliminate it if more current is needed.
 */
#define SENTRY_CHASSIS_ENABLE_CLIP                  TRUE

#if SENTRY_CHASSIS_ENABLE_CLIP
#define SENTRY_CHASSIS_MAX_CURRENT 5000  // mA
#endif

/**
 * @name SentryChassis
 * @brief interface to process chassis motor feedback and send target current.
 * @pre hardware is properly set. CAN id of each motor should be the same as chassis_motor_id_t.
 * @usage 1. init(CANInterface *). The interface should be properly initialized.
 *        2. control the data flow based on actual implementation
 */
class SentryChassis {

public:

    
    static float constexpr chassis_motor_decelerate_ratio = 19.2f; // 3591/187 on the data sheet

    enum motor_id_t {
        MOTOR_RIGHT,
        MOTOR_LEFT,
        MOTOR_COUNT // = 2
    };

    // Structure for each motor
    struct motor_t {

        motor_id_t id;

        int16_t actual_rpm_raw;
        int16_t actual_current_raw;
        uint8_t actual_temperature_raw;

        int16_t actual_angle = 0;
        int16_t last_angle_raw = 0; // 8192 for 360 degrees

        float actual_angular_velocity; // degree/s
        int round_count = 0;
        int target_current;

        float present_position;
        float present_velocity;
    };

    static motor_t motor[];

    /**
     * @brief send all target currents
     * @return
     */
    static bool send_currents();

    /**
     * @brief set CAN interface for receiving and sending
     * @param can_interface
     */
    static void init(CANInterface* can_interface);



private:

    static CANInterface* can;

    static void process_feedback(CANRxFrame const*rxmsg);

    static float constexpr displacement_per_round = 17.28f;

    friend CANInterface;

};


#endif //META_INFANTRY_SENTRY_CHASSIS_H
