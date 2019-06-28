//
// Created by liuzikai on 2018-12-29.
// Zhu Kerui wrote code about processing gimbal feedback and the bullet control.
// Feng Chuhao wrote code about sending gimbal currents.
//

/**
 * This file contains Gimbal Interface module.
 *
 * @note This interface support both RM6623 and GM6020 motor, if their CAN ID and field number are configured properly.
 *       For example, when RM6623 has motor ID 5, and GM6020 has motor ID 1, they both receive 0x1FF field 1 and
 *       send feedback of 0x205.
 */

#ifndef META_INFANTRY_GIMBAL_INTERFACE_H
#define META_INFANTRY_GIMBAL_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

/* Board Guard */
#if defined(BOARD_RM_2018_A)
#elif defined(BOARD_RM_2017)
#else
#error "GimbalIF has not been defined for selected board"
#endif


/**
 * Enable velocity calculation from feedback angle (using derivative.)
 */
#define GIMBAL_INTERFACE_ENABLE_VELOCITY_CALCULATION  TRUE

/**
 * Enable clip at the moment of sending current.
 * Only for safety. There is NO signal for clipping. Be sure to eliminate it if more current is needed.
 */
#define GIMBAL_INTERFACE_ENABLE_CLIP  FALSE

#if GIMBAL_INTERFACE_ENABLE_CLIP
#define GIMBAL_INTERFACE_MAX_CURRENT 5000
#define GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT 5000
#define GIMBAL_INTERFACE_BULLET_PLATE_MAX_CURRENT 5000
#endif


class GimbalBase {
public:
    enum motor_id_t {
        YAW = 0,
        PITCH = 1,
        BULLET = 2,
        PLATE = 3,
        MOTOR_COUNT = 4
    };
};


/**
 * @name GimbalInterface
 * @brief Interface to interact with Yaw, Pitch, Bullet Loader (using CAN) and friction wheels (by PWM). Maintain the
 *        feedback info and provide method to send control signal
 * @pre Hardware is connected properly (see ONES doc)
 * @pre PWM pins are set properly in board.h (I5 - alt 3, I6 - alt 3)
 * @usage 1. Call init(CANInterface *). The interface should be properly initialized.
 *        2. Read feedback from variables.
 *           Write target current / duty cycle to variables, then call send_gimbal_currents to apply
 */
class GimbalIF : public GimbalBase {

public:

    /**
     * @brief Initialize the gimbal
     * @param can_interface           initialized CANInterface for yaw, pitch and bullet_loader motor
     * @param yaw_front_angle_raw     raw angle of yaw when gimbal points straight forward, depending on installation.
     * @param pitch_front_angle_raw   raw angle of pitch when gimbal points straight forward, depending on installation.
     */
    static void init(CANInterface *can_interface, uint16_t yaw_front_angle_raw, uint16_t pitch_front_angle_raw);


    struct motor_feedback_t {

    public:

        motor_id_t id;

        /**
         * Normalized Angle and Rounds
         *  Using the front angle_raw as reference.
         *  Range: -180.0 (clockwise) to 180.0 (counter-clockwise)
         */

        float actual_angle = 0.0f;     // [degree]
#if GIMBAL_INTERFACE_ENABLE_VELOCITY_CALCULATION
        float actual_velocity = 0.0f;  // [degree/s]
#endif
        int actual_current = 0;
        int round_count = 0;

        time_msecs_t last_update_time = 0;

        /**
         * @brief set current actual angle as the front angle
         */
        void reset_front_angle();

    private:

        uint16_t last_angle_raw = 0;  // in the range of [0, 8191]

#if GIMBAL_INTERFACE_ENABLE_VELOCITY_CALCULATION
        // Variable for velocity sampling
        time_msecs_t sample_time_stamp = 0;
        int sample_count = 0;
        int sample_movement_sum = 0;
#endif

        friend GimbalIF;
        friend int main();

    };

    /**
     * @brief Motor feedback structure
     */
    static motor_feedback_t feedback[MOTOR_COUNT];


    /**
     * @brief target current array in the order defined in motor_id_t
     */
    static int target_current[MOTOR_COUNT];

    /**
     * @brief friction_wheels duty cycle
     */
    static float fw_duty_cycle;

    /**
     * @brief send target_current of each motor
     * @return whether currents are sent successfully
     */
    static void send_gimbal_currents();


private:

    static CANInterface *can_;

    static void process_motor_feedback(CANRxFrame const *rxmsg);
    friend CANInterface;

#if GIMBAL_INTERFACE_ENABLE_VELOCITY_CALCULATION
    static constexpr int VELOCITY_SAMPLE_INTERVAL = 50;  // count of feedback for one sample of angular velocity
#endif

    enum friction_wheel_channel_t {
        FW_LEFT = 0,  // The left  friction wheel, PI5, channel 0
        FW_RIGHT = 1  // The right friction wheel, PI6, channel 1
    };

};


#endif //META_INFANTRY_GIMBAL_INTERFACE_H
