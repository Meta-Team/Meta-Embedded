//
// Created by liuzikai on 2019-04-12.
// Modified by zhukerui and jintengjun
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
#define SENTRY_CHASSIS_ENABLE_CLIP  FALSE

#if SENTRY_CHASSIS_ENABLE_CLIP
#define SENTRY_CHASSIS_MAX_CURRENT 5000  // mA
#endif

class SChassisBase {
public:
    enum motor_id_t {
        MOTOR_RIGHT,
        MOTOR_LEFT,
        MOTOR_COUNT
    };
};

/**
 * @name SChassisIF
 * @note "IF" stands for "interface", "S" is short for "Sentry"
 * @brief Interface to process chassis motor feedback and send target current.
 * @pre Hardware is properly set. CAN id of each motor should be the same as motor_id_t.
 * @usage 1. Call init(CANInterface *). The interface should be properly initialized.
 *        2. Read feedback from variables.
 *           Write target current to variables, then call send_currents() to apply changes
 * @note This module is designed to process feedback automatically, but not to send current automatically, to avoid
 *       unintended chassis movements.
 */
class SChassisIF : public SChassisBase{

public:

    static float present_position();
    static float present_velocity();

    static void clear_position();

    struct motor_feedback_t {

        motor_id_t id;

        float present_position;   // [cm]
        float present_velocity;   // [cm/s]

        time_msecs_t last_update_time = 0;

    private:

        int16_t last_angle_raw = 0; // 8192 for 360 degrees

        void clear_position(){
            present_position = 0;
        }

        friend SChassisIF;
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
     * @brief set CAN interface for receiving and sending
     * @param can_interface
     */
    static void init(CANInterface* can_interface);

    static bool send_currents();

private:

    static CANInterface* can;
    static void process_feedback(CANRxFrame const*rxmsg);
    friend CANInterface;

    static float constexpr DISPLACEMENT_PER_ROUND = 17.28f;
    static float constexpr CHASSIS_MOTOR_DECELERATE_RATIO = 19.2f; // 3591/187 on the datasheet
};


#endif //META_INFANTRY_SENTRY_CHASSIS_H