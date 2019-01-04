//
// Created by liuzikai on 2018-12-29.
// Zhu Kerui wrote code about processing gimbal feedback.
// Feng Chuhao wrote code about sending gimbal currents.
//

#ifndef META_INFANTRY_GIMBAL_H
#define META_INFANTRY_GIMBAL_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

/**
 * Enable clip at the moment of sending current.
 * Only for safety. There is NO signal for clipping. Be sure to eliminate it if more current is needed.
 */
#define GIMBAL_INTERFACE_ENABLE_CLIP                  TRUE

#if GIMBAL_INTERFACE_ENABLE_CLIP
#define GIMBAL_INTERFACE_MAX_CURRENT 5000
#endif


class GimbalInterface {

public:

    typedef enum {
        YAW_ID = 0,
        PIT_ID = 1
    } motor_id_t;

    /** Motor Interface **/
    typedef struct motor {

    public:

        motor_id_t id;
        bool enabled;  // if not enabled, 0 current will be sent in send_gimbal_currents

        // +: clockwise, -: counter-clockwise
        int target_current;

        /**
         * Normalized Angle and Rounds
         *  Using the front angle_raw as reference.
         *  Range: -180.0 (clockwise) to 180.0 (counter-clockwise)
         */
        float actual_angle; // the actual angle of the gimbal, compared with the front
        int round_count;  // the rounds that the gimbal turns

        float angular_velocity;  // instant angular velocity [degree/s], positive when counter-clockwise, negative otherwise

        int actual_current;  // feedback current

        // Set current angle as the front angle
        void reset_front_angle() {
            actual_angle = 0;
            round_count = 0;
        }

        // Get total angle from the original front angle
        float get_accumulate_angle() {
            return actual_angle + round_count * 360.0f;
        }

    private:

        uint16_t last_angle_raw;  // the raw angle of the newest feedback, in [0, 8191]

        // For velocity sampling
        int sample_count;
        int sample_movement_sum;
        time_msecs_t sample_time;  // last sample time, for velocity calculation

        friend GimbalInterface;

    } motor_t;

    static motor_t yaw;
    static motor_t pitch;

    /**
     * @brief send target_current of each motor
     * @return
     */
    static bool send_gimbal_currents();

    /**
     * @brief process CAN rx frame
     * @param rxmsg
     * @return whether the rx frame is from gimbal motors
     */
    static bool process_motor_feedback (CANRxFrame *rxmsg);

    /**
     * Default constructor
     */
    GimbalInterface() {
        yaw.id = YAW_ID;
        yaw.enabled = false;
        yaw.sample_time = 0;
        yaw.sample_count = 0;
        yaw.sample_movement_sum = 0;
        yaw.actual_angle = 0;
        yaw.last_angle_raw = 0;

        pitch.id = PIT_ID;
        pitch.enabled = false;
        pitch.sample_time = 0;
        pitch.sample_count = 0;
        pitch.sample_movement_sum = 0;
        pitch.actual_angle = 0.0;
        pitch.last_angle_raw = 0;
    }

    /**
     * @brief set the CAN interface
     * @param can_interface
     */
    static void set_can_interface (CANInterface* can_interface) {
        can = can_interface;
    }


private:

    static CANInterface* can;

    // Count of feedback for one sample of angular velocity
    static constexpr int velocity_sample_interval = 100;

};


#endif //META_INFANTRY_GIMBAL_H
