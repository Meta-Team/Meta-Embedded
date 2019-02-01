//
// Created by liuzikai on 2018-12-29.
// Zhu Kerui wrote code about processing gimbal feedback.
// Feng Chuhao wrote code about sending gimbal currents.
//

#ifndef META_INFANTRY_GIMBAL_INTERFACE_H
#define META_INFANTRY_GIMBAL_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"
#include "pid_controller.h"

/**
 * Enable clip at the moment of sending current.
 * Only for safety. There is NO signal for clipping. Be sure to eliminate it if more current is needed.
 */
#define GIMBAL_INTERFACE_ENABLE_CLIP                  TRUE

#if GIMBAL_INTERFACE_ENABLE_CLIP
#define GIMBAL_INTERFACE_MAX_CURRENT 5000
#define GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT 3000
#endif

/**
 * @name GimbalInterface
 * @brief interface to process feedback from gimbal and send control signals to gimbal, including Yaw, Pitch, Bullet
 *        Loader (using CAN) and friction wheels (by PWM).
 * @pre Hardware is set properly (CAN id of Yaw = 5 and Pitch = 6, CAN id of bullet loader (C610) = 7, friction wheels
 *      left = PI5, right = PI6.
 * @pre PWM pin is set properly in board.h (I5 - alt 3, I6 - alt 3)
 * @pre start(CANInterface *). The interface should be properly initialized.
 */
class GimbalInterface {

public:

    int remained_bullet;

    typedef enum {
        YAW_ID = 0,
        PIT_ID = 1,
        BULLET_LOADER_ID = 2
    } motor_id_t;

    /**
     * Motor Interface
     * which is used for yaw, pitch and bullet loader motors
     */
    typedef struct {

    public:

        motor_id_t id;
        bool enabled = false;  // if not enabled, 0 current will be sent in send_gimbal_currents

        // +: clockwise, -: counter-clockwise
        int target_current = 0;  // the current that we want the motor to have
        int sent_current = 0;  // the required current we send in order to reach stability around the target current

        /**
         * Normalized Angle and Rounds
         *  Using the front angle_raw as reference.
         *  Range: -180.0 (clockwise) to 180.0 (counter-clockwise)
         */
        float actual_angle = 0.0f; // the actual angle of the gimbal, compared with the front
        float angular_velocity = 0.0f;  // instant angular velocity [degree/s], positive when counter-clockwise, negative otherwise
        int actual_current = 0;  // feedback current
        int round_count = 0;  // the rounds that the gimbal turns

        // Set current angle as the front angle
        void reset_front_angle() {
            motor_t::actual_angle = 0;
            motor_t::round_count = 0;
        }

        // Get total angle from the original front angle
        float get_accumulate_angle() {
            return motor_t::actual_angle + motor_t::round_count * 360.0f;
        }

        // For velocity sampling and gimbal feedback module
        time_msecs_t sample_time = 0;  // last sample time, for velocity calculation

        // PID control for the motor
        PIDController motor_PIDController;

    private:

        uint16_t last_angle_raw = 0;  // the raw angle of the newest feedback, in [0, 8191]

        // For velocity sampling
        int sample_count = 0;
        int sample_movement_sum = 0;

        friend GimbalInterface;

    } motor_t;

    static motor_t yaw;
    static motor_t pitch;
    static motor_t bullet_loader;

    /**
     * Friction Wheels Interface
     * control the two friction wheels that shoot the bullets
     */
    typedef struct {

        bool enabled = false;

        float duty_cycle = 0.0f;

    } friction_wheels_t;

    static friction_wheels_t friction_wheels;

    /**
     * @brief set the CAN interface, start PWM driver and set the PID
     * @param can_interface
     */
    static void start(CANInterface *can_interface);

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
    static bool process_motor_feedback(CANRxFrame *rxmsg);


    /**PID Functions**/

    /**
     * @brief set the motor PID parameters
     * @param motor
     * @param _kp
     * @param _ki
     * @param _kd
     * @param _i_limit
     * @param _out_limit
     */
    static void set_motor_PID(motor_t* motor, float _kp, float _ki, float _kd, float _i_limit, float _out_limit);

    /**
     * @brief calculate the needed output
     * @param motor
     * @param now
     * @param target
     * @return
     */
    static int calc_motor_current(motor_t* motor, float now, float target);

    /**
     * Default constructor
     */
    GimbalInterface() {
        yaw.id = YAW_ID;
        pitch.id = PIT_ID;
        bullet_loader.id = BULLET_LOADER_ID;

    }

private:

    static CANInterface *can;

    // Count of feedback for one sample of angular velocity
    static constexpr int velocity_sample_interval = 50;

    static constexpr PWMDriver *friction_wheel_pwm_driver = &PWMD8;

    // TODO: determine the install order of left and right wheels
    enum friction_wheel_channel_t {
        FW_LEFT = 0,  // The left friction wheel, PI5, channel 0
        FW_RIGHT = 1  // The right friction wheel, PI6, channel 1
    };

};


#endif //META_INFANTRY_GIMBAL_INTERFACE_H
