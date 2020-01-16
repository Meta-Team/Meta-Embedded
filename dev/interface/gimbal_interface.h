//
// Created by liuzikai on 2018-12-29.
// Zhu Kerui wrote code about processing gimbal feedback and the bullet control.
// Feng Chuhao wrote code about sending gimbal currents.
// Mo Kanya wrote code about sending friction wheels' currents and processing friction wheels' feedback
// Qian Chen wrote about motor can channel distribution mechanism.
//

/**
 * @file    gimbal_interface.h
 * @brief   Interface to interact with low level driver of gimbal, including processing chassis motor feedback and
 *          sending target currents.
 *
 * @addtogroup gimbal
 * @{
 */

#ifndef META_INFANTRY_GIMBAL_INTERFACE_H
#define META_INFANTRY_GIMBAL_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

/// Board Guard
#if defined(BOARD_RM_2018_A)
#elif defined(BOARD_RM_2017)
#else
#error "GimbalIF has not been defined for selected board"
#endif


/**
 * @name GimbalBase
 * @brief provide shared index enum to gimbal-related class
 */
class GimbalBase {
public:
    enum motor_id_t {
        YAW = 0,
        PITCH = 1,
        BULLET = 2,
        PLATE = 3,
        FW_LEFT = 4,
        FW_RIGHT = 5,
        MOTOR_COUNT = 6
    };
};


/**
 * @name GimbalIF
 * @note "IF" stands for "interface"
 * @brief Interface to interact with Yaw, Pitch, Bullet Loader (using CAN) and friction wheels (by PWM). Maintain the
 *        feedback info and provide method to send control signal
 * @pre Hardware is connected properly (see ONES doc)
 * @pre PWM pins are set properly in board.h (I5 - alt 3, I6 - alt 3)
 * @usage 1. Call init(CANInterface *). The interface should be properly initialized.
 *        2. Read feedback from variables.
 *           Write target current / duty cycle to variables, then call send_gimbal_currents to apply changes
 * @note This module is designed to process feedback automatically, but not to send current automatically, to avoid
 *       unintended gimbal movements.
 * @note About coordinate: all components in this module use original coordinate of EACH motor. DO NOT directly add
 *       negative sign to code in this module.
 * @note This interface support both RM6623 and GM6020 motor, if their CAN ID and field number are configured properly.
 *       For example, when RM6623 has motor ID 5, and GM6020 has motor ID 1, they both receive 0x1FF field 1 and
 *       send feedback of 0x205.
 */


/**
 * OPTION: Enable velocity calculation from feedback angle (using derivative.)
 */
#define GIMBAL_INTERFACE_ENABLE_VELOCITY_DIFFERENTIAL  TRUE

/**
 * OPTION: Enable clip at the moment of sending current.
 * @note Only for safety. There is NO signal for clipping. Be sure to eliminate it if more current is needed.
 */
#define GIMBAL_INTERFACE_ENABLE_CLIP  FALSE


#if GIMBAL_INTERFACE_ENABLE_CLIP
#define GIMBAL_INTERFACE_MAX_CURRENT 5000
#define GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT 5000
#define GIMBAL_INTERFACE_BULLET_PLATE_MAX_CURRENT 5000
#endif

class GimbalIF : public GimbalBase {

public:

    enum motor_type_t {
        NONE_MOTOR,
        RM6623,
        M2006,
        GM6020,
        GM3510,
        M3508
    };

    enum motor_can_channel_t {
        NONE,
        can_channel_1,
        can_channel_2
    };

    /**
     * Initialize GimbalIF. Angles of bullet loader and bullet plate will be reset.
     * @param can1_interface           Initialized CANInterface for yaw, pitch and bullet_loader motor
     * @param yaw_front_angle_raw     Raw angle of yaw when gimbal points straight forward, depending on installation.
     * @param pitch_front_angle_raw   Raw angle of pitch when gimbal points straight forward, depending on installation.
     * @param yaw_type                Yaw motor motor type
     * @param pitch_type              Pitch motor motor type
     * @param bullet_type             Bullet loader motor type
     * @param plate_type              Bullet plate motor type, default to NONE_MOTOR
     */
    static void init(CANInterface *can1_interface,                      CANInterface *can2_interface,
                     uint16_t yaw_front_angle_raw,                      uint16_t pitch_front_angle_raw,

                     motor_type_t yaw_type,                             motor_type_t pitch_type,
                     motor_type_t bullet_type,                          motor_type_t plate_type,

                     motor_can_channel_t yaw_can_channel,               motor_can_channel_t pitch_can_channel,
                     motor_can_channel_t bullet_can_channel,            motor_can_channel_t plate_can_channel);


    struct motor_feedback_t {

    public:

        void init(motor_type_t type_, motor_id_t id_);

        motor_type_t type;
        motor_id_t id;

        motor_can_channel_t can_channel;

        /**
         * Normalized angle
         * @note Viewing from TOP of 6623/2006 motor. 180 <--CCW-- front_angle_raw --CW--> -180
         */
        float actual_angle = 0.0f;     // [degree]

        /**
         * Velocity
         * @note Viewing from TOP of 6623/2006 motor. Positive for CCW. Negative for CW.
         */
        float actual_velocity = 0.0f;  // [degree/s]

        /**
         * Actual current
         * @note Direction is UNKNOWN yet. In reality, it vibrates significantly, and it's not useful for now.
         */
        int actual_current = 0;  // [mA]

        /**
         * Number of round
         * @note Viewing from TOP of 6623/2006 motor. Positive for CCW. Negative for CW.
         */
        int round_count = 0;

        /**
         * Last update time (ms, from system start)
         */
        time_msecs_t last_update_time = 0;

        /**
         * Set current actual angle as the zero reference angle and clear the round count (accumulated angle = 0)
         */
        void reset_front_angle();

        /**
         * Get total angle from the original front angle
         * @return Accumulated angle since last reset_front_angle [degree, positive for CCW viewing from top]
         */
        float accumulated_angle();

    private:

        uint16_t last_angle_raw = 0;  // in the range of [0, 8191]

#if GIMBAL_INTERFACE_ENABLE_VELOCITY_DIFFERENTIAL
        // Variables for velocity sampling
        time_msecs_t sample_time_stamp = 0;
        int sample_count = 0;
        int sample_movement_sum = 0;
#endif

        friend GimbalIF;
        friend int main();

    };

    /**
     * Motor feedback structure
     */
    static motor_feedback_t feedback[MOTOR_COUNT];


    /**
     * Target current array in the order defined in motor_id_t
     */
    static int target_current[MOTOR_COUNT];

    /**
     * Friction wheels duty cycle
     */
    static float fw_duty_cycle;

    /**
     * Send target_current of each motor
     * @return Whether currents are sent successfully
     */
    static void send_gimbal_currents();


private:

    static CANInterface *can1_;
    static CANInterface *can2_;

    static CANTxFrame can1_txmsg;
    static CANTxFrame can2_txmsg;
    static CANTxFrame fw_txmsg;

    static void process_can1_motor_feedback(CANRxFrame const *rxmsg);  // callback function
    static void process_can2_motor_feedback(CANRxFrame const *rxmsg);  // callback function

    friend CANInterface;

#if GIMBAL_INTERFACE_ENABLE_VELOCITY_DIFFERENTIAL
    static constexpr int VELOCITY_SAMPLE_INTERVAL = 50;  // count of feedback for one sample of angular velocity
#endif

//    enum friction_wheel_channel_t {
//        FW_LEFT = 0,  // The left  friction wheel, PI5, channel 0
//        FW_RIGHT = 1  // The right friction wheel, PI6, channel 1
//    };

};


#endif //META_INFANTRY_GIMBAL_INTERFACE_H

/** @} */