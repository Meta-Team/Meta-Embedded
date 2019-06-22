//
// Created by zhukerui on 2019/6/8.
//

#ifndef META_INFANTRY_SUSPENSION_GIMBAL_INTERFACE_H
#define META_INFANTRY_SUSPENSION_GIMBAL_INTERFACE_H
#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"
#include "ahrs_ext.h"
#include "ahrs_math.hpp"

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
#define SUSPENSION_GIMBAL_INTERFACE_ENABLE_CLIP TRUE

#if SUSPENSION_GIMBAL_INTERFACE_ENABLE_CLIP
#define SUSPENSION_YAW_MAX_VOLTAGE 30000
#define SUSPENSION_PITCH_MAX_VOLTAGE 29000
#define SUSPENSION_GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT 3000
#define MAX_YAW_ANGLE 170.0f // degree
#define MIN_YAW_ANGLE -170.0f
#define MAX_PITCH_ANGLE 40.0f // degree
#define MIN_PITCH_ANGLE -85.0f
#define BULLET_LOADER_SPEED 360.0f // degree/s
#endif

/**
 * @name SuspensionGimbalIF
 * @brief Interface to process feedback from suspension gimbal and send control signals to gimbal, including Yaw, Pitch, Bullet
 *        Loader (using CAN) and friction wheels (by PWM).
 * @pre Hardware is connected properly (see ONES doc)
 * @pre PWM pins are set properly in board.h (I5 - alt 3, I6 - alt 3)
 * @usage 1. Call init(CANInterface *). The interface should be properly initialized.
 *        2. Read feedback from variables.
 *           Write target voltage / target current / duty cycle to variables, then call send_gimbal_currents.
 */

typedef enum {
    OFF = 0,
    AWAIT = 1,
    SHOOT = 2,
} shoot_mode_t;

class SuspensionGimbalIF {

public:

    typedef enum {
        YAW_ID = 0,
        PIT_ID = 1,
        BULLET_LOADER_ID = 2
    } motor_id_t;

    static shoot_mode_t shoot_mode;

    /**
     * Interface for each motor, which is used for yaw, pitch and bullet loader motors
     */
    struct MotorInterface {

        motor_id_t id;

        bool enabled = false;  // if not enabled, 0 current will be sent in send_gimbal_currents

        float angular_velocity = 0.0f;  // [degree/s]

        float angular_position; // [degree]

        int16_t target_signal = 0;  // the current/voltage that we want the motor to have

        float last_angle = 0.0f;  // the raw angle of the newest feedback, in [0, 360]

        void initializer(motor_id_t id_, float movement_LB, float movement_UB, float DR){
            id = id_;
            angle_movement_lower_bound = movement_LB;
            angle_movement_upper_bound = movement_UB;
            deceleration_ratio = DR;
        }

    private:
        /**
         * Normalized Angle and Rounds
         */
        float target_angle = 0;

        // Some const parameters for feedback processing
        float angle_movement_lower_bound;
        float angle_movement_upper_bound;
        float deceleration_ratio;

        /**
         * @brief set current actual angle as the front angle
         */
        void reset_front_angle(){
            angular_position = 0;
            target_angle = 0;
        }

        friend SuspensionGimbalIF;
        friend class SuspensionGimbalSKD;
    };
    static MotorInterface yaw;
    static MotorInterface pitch;
    static MotorInterface bullet_loader;

    /**
     * @brief set the CAN interface, start PWM driver and set the PID
     * @param can_interface
     * @param yaw_front_angle_raw   raw angle of yaw when gimbal points straight forward, depending on installation.
     * @param pitch_front_angle_raw   raw angle of pitch when gimbal points straight forward, depending on installation.
     */
    static void init(CANInterface *can_interface, AHRSExt *ahrsExt, float yaw_front_angle_raw = 0, float pitch_front_angle_raw = 0);

    /**
     * @brief send target_current of each motor
     * @return
     */
    static bool send_gimbal_currents();

private:

    enum friction_wheel_channel_t {
        FW_LEFT = 0,  // The left friction wheel, PI5, channel 0
        FW_RIGHT = 1  // The right friction wheel, PI6, channel 1
    };

    static float pitchFront;

    static float shoot_duty_cycles[3];  // the array contains the duty cycles for different shoot modes

    static CANInterface *can_;

    static AHRSExt *ahrs_;

    friend CANInterface;
    friend class SuspensionGimbalSKD;

    /**
     * @brief process CAN rx frame
     * @param rxmsg
     */
    static void process_motor_feedback(CANRxFrame const *rxmsg);

    // Count of feedback for one sample of angular velocity
    static constexpr int VELOCITY_SAMPLE_INTERVAL = 50;

    static constexpr PWMDriver *FRICTION_WHEEL_PWM_DRIVER = &PWMD8;

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


#endif //META_INFANTRY_SUSPENSION_GIMBAL_INTERFACE_H
