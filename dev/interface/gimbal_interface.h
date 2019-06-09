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
#error "GimbalInterface has not been defined for selected board"
#endif

/*
 * Enable clip at the moment of sending current.
 * Only for safety. There is NO signal for clipping. Be sure to eliminate it if more current is needed.
 */
#define GIMBAL_INTERFACE_ENABLE_CLIP TRUE

#if GIMBAL_INTERFACE_ENABLE_CLIP
#define GIMBAL_INTERFACE_MAX_CURRENT 5000
#define GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT 3000
#endif

/**
 * @name GimbalInterface
 * @brief Interface to process feedback from gimbal and send control signals to gimbal, including Yaw, Pitch, Bullet
 *        Loader (using CAN) and friction wheels (by PWM).
 * @pre Hardware is connected properly (see ONES doc)
 * @pre PWM pins are set properly in board.h (I5 - alt 3, I6 - alt 3)
 * @usage 1. Call init(CANInterface *). The interface should be properly initialized.
 *        2. Read feedback from variables.
 *           Write target current / duty cycle to variables, then call send_gimbal_currents.
 */
class GimbalInterface {

public:

    typedef enum {
        YAW_ID = 0,
        PIT_ID = 1,
        BULLET_LOADER_ID = 2
    } motor_id_t;

    /**
     * Interface for each motor, which is used for yaw, pitch and bullet loader motors
     */
    class MotorInterface {

    public:

        motor_id_t id;

        bool enabled = false;  // if not enabled, 0 current will be sent in send_gimbal_currents

        // +: clockwise, -: counter-clockwise
        int target_current = 0;  // the current that we want the motor to have

        /**
         * Normalized Angle and Rounds
         *  Using the front angle_raw as reference.
         *  Range: -180.0 (clockwise) to 180.0 (counter-clockwise)
         */
        float actual_angle = 0.0f; // the actual angle of the gimbal, compared with the front
        float angular_velocity = 0.0f;  // instant angular velocity [degree/s], positive when counter-clockwise, negative otherwise
        int actual_current = 0;  // feedback current
        int round_count = 0;  // the rounds that the gimbal turns

        /**
         * @brief set current actual angle as the front angle
         */
        void reset_front_angle();

    private:

        uint16_t last_angle_raw = 0;  // the raw angle of the newest feedback, in [0, 8191]

        // For velocity sampling
        time_msecs_t sample_time = 0;  // last sample time, for velocity calculation
        int sample_count = 0;
        int sample_movement_sum = 0;

        friend GimbalInterface;
        friend int main();

    };
    static MotorInterface yaw;
    static MotorInterface pitch;
    static MotorInterface bullet_loader;

    /**
     * Friction Wheels Interface
     * control the two friction wheels that shoot the bullets
     */
    class FrictionWheelsInterface {
    public:
        bool enabled = false;
        float duty_cycle = 0.0f;
    };
    static FrictionWheelsInterface friction_wheels;

    /**
     * @brief set the CAN interface, start PWM driver and set the PID
     * @param can_interface
     * @param yaw_front_angle_raw   raw angle of yaw when gimbal points straight forward, depending on installation.
     * @param pitch_front_angle_raw   raw angle of pitch when gimbal points straight forward, depending on installation.
     */
    static void init(CANInterface *can_interface, uint16_t yaw_front_angle_raw = 0, uint16_t pitch_front_angle_raw = 0);

    /**
     * @brief send target_current of each motor
     * @return
     */
    static bool send_gimbal_currents();

    static float absolute_angle;  // the absolute angle of the gimbal measured by the magnetometer, only used for YAW


private:

    static CANInterface *can_;

    /**
     * @brief process CAN rx frame
     * @param rxmsg
     */
    static void process_motor_feedback(CANRxFrame const *rxmsg);

    friend CANInterface;

    // Count of feedback for one sample of angular velocity
    static constexpr int VELOCITY_SAMPLE_INTERVAL = 50;

    static constexpr PWMDriver *FRICTION_WHEEL_PWM_DRIVER = &PWMD8;

    enum friction_wheel_channel_t {
        FW_LEFT = 0,  // The left friction wheel, PI5, channel 0
        FW_RIGHT = 1  // The right friction wheel, PI6, channel 1
    };

    static constexpr PWMConfig FRICTION_WHEELS_PWM_CFG = {
            50000,   // frequency
            1000,    // period
            nullptr, // callback
            {
                    {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH0
                    {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH1
                    {PWM_OUTPUT_DISABLED, nullptr},    // CH2
                    {PWM_OUTPUT_DISABLED, nullptr}     // CH3
            },
            0,
            0
    };

};


#endif //META_INFANTRY_GIMBAL_INTERFACE_H
