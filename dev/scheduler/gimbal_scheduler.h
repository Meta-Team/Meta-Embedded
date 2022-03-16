//
// Created by liuzikai on 2019-01-05.
// Rebuild by Chen Qian on 2022-03-16.
//

/**
 * @file    gimbal_scheduler.h
 * @brief   Scheduler to control gimbal to meet the target, including a thread to invoke PID calculation in period.
 *
 * @version 3.0a
 * @date 02.28, 2022
 * @addtogroup gimbal
 * @{
 */

#ifndef META_INFANTRY_GIMBAL_CONTROLLER_H
#define META_INFANTRY_GIMBAL_CONTROLLER_H

#include "ch.hpp"
#include "hardware_conf.h"

#include "can_motor_scheduler.h"
#include "ahrs_abstract.h"
#include "scheduler_base.h"

#include <cmath>

#if defined(HERO)
/// Enable sub-pitch motor in gimbal scheduler
#define ENABLE_SUBPITCH
#endif

/**
 * @name GimbalSKD
 * @note SKD stands for "scheduler"
 * @brief Scheduler to control gimbal to meet the target, including a thread to invoke PID calculation in period.
 * @pre GimbalIF and ChassisIF have been initialized properly
 * @usage 1. start(), load_pid_params()\n
 *        2. set_mode() and set_target_angle() as needed\n
 *        3. Leave the rest of the work to this SKD
 * @note ABOUT COORDINATE
 *       You can switch the coordinate between chassis coordinate and gimbal coordinate. The direction definition:
 *       Yaw: CCW as positive, Pitch: Up as positive.
 * @note ABOUT TARGET ANGLES
 *       APIs use angles of GIMBAL (rather than those of motors). But for local storage, deceleration_ratio is included.
 *       Conversions are performed when APIs are invoked.
 */

class GimbalSKD : public SKDBase {

public:
    enum angle_id_t{
        YAW,
        PITCH,
#if ENABLE_SUBPITCH == TRUE
        SUB_PITCH,
#endif
        GIMBAL_MOTOR_COUNT
    };

    enum install_direction_t {
        POSITIVE = 1, ///<CCW for motor
        NEGATIVE = -1 ///<CW for motor
    };

    /**
     * @brief Start this scheduler
     * @param gimbal_ahrs_                [in] Pointer to an initialized AHRS on gimbal
     * @param ahrs_angle_rotation_        [in] Rotation matrix for AHRS angle
     * @param ahrs_gyro_rotation_         [in] Rotation matrix for AHRS gyro
     * @param thread_prio                 [in] Priority of PID calculating thread
     */
    static void start(AbstractAHRS *gimbal_ahrs_, const Matrix33 ahrs_angle_rotation_, const Matrix33 ahrs_gyro_rotation_, tprio_t thread_prio);

    /**
     * @brief Set mode of this SKD
     * @param skd_mode [in] The current mode of gimbal scheduler.
     */
    static void set_mode(mode_t skd_mode);

    /**
     * @brief Get mode of this SKD
     * @return Current mode
     */
    static mode_t get_mode();

    /**
     * @brief Set target angles
     * @param yaw_target_angle          [in] GIMBAL yaw target ACCUMULATED angle on ground coordinate [degree]
     * @param pitch_target_angle        [in] GIMBAL pitch target ACCUMULATED angle on ground coordinate [degree]
     * @param sub_pitch_target_angle    [in] GIMBAL sub-pitch target ACCUMULATED angle on pitch coordinate [degree]
     */
    static void set_target_angle(float yaw_target_angle, float pitch_target_angle, float sub_pitch_target_angle = 0);

    /**
     * @brief Get current target angle data
     * @param angleID [in] YAW or PITCH or SUB_PITCH
     * @return Current target angle of GIMBAL
     */
    static float get_target_angle(GimbalSKD::angle_id_t angleID);

    /**
     * @brief Get AHRS feedback angle
     * @param angleID [in] (YAW/PITCH) GimbalSKD::angle_id_t
     * @return AHRS angle of YAW/PITCH
     */
    static float get_feedback_angle(GimbalSKD::angle_id_t angleID);

    /// TODO: Re-enable shell functions
//    static void cmdFeedback(void *);
    static const Shell::Command shellCommands[];

    /**
     * @brief When using AHRS feedback and find the positive direction of motor and AHRS are different, use this.
     * @param angle             [in] The axis needs to be inverted.
     * @param install_direction [in] The install direction of angle.
     */
    static void set_installation(GimbalSKD::angle_id_t angle, GimbalSKD::install_direction_t install_direction);

private:
    /*===========================================================================*/
    /*                             AHRS Related Var                              */
    /*===========================================================================*/

    // The reason why there are two extra matrices is that the AHRS system is only stable in certain installation with
    // specific matrix, but the output may not be the angles we want. So we add two matrices to mapping the output angle
    // and velocity to our desired feedback angle.

    /**
     * @brief AHRS module address
     */
    static AbstractAHRS *gimbal_ahrs;

    /**
     * @brief Rotation matrix for angle of AHRS data output.
     */
    static Matrix33 ahrs_angle_rotation;

    /**
     * @brief Rotation matrix for angular velocity of AHRS data output.
     */
    static Matrix33 ahrs_gyro_rotation;

    /*===========================================================================*/
    /*                             Motor Related Var                             */
    /*===========================================================================*/

    /**
     * @brief Custom feedback angle for gimbal (mainly for AHRS)
     */
    static float feedback_angle[GIMBAL_MOTOR_COUNT];

    /**
     * @brief Custom feedback angular velocity for gimbal (mainly for AHRS)
     */
    static float feedback_velocity[GIMBAL_MOTOR_COUNT];

#ifdef PARAM_ADJUST
    static bool a2v_pid_enabled;  // only allowed by PAUser
#endif


    /*===========================================================================*/
    /*                            Control Related Var                            */
    /*===========================================================================*/

    /**
     * @brief [deg] Angles of motors.
     */
    static float target_angle[GIMBAL_MOTOR_COUNT];

    /**
     * @brief [deg] Last angle for updating AHRS accumulated angle.
     */
    static float last_angle[GIMBAL_MOTOR_COUNT];

    /**
     * @brief Interval for scheduler calculation thread.
     */
    static constexpr unsigned int SKD_THREAD_INTERVAL = 1; // PID calculation interval [ms]

    /**
     * @brief Scheduler thread, for calculation.
     */
    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skd_thread;

    /**
     * @brief Gimbal Behavior Control
     */
    static mode_t mode;

    /**
     * For inverted installation of gimbal motors and using AHRS feedback.
     */
    static install_direction_t install_direction[GIMBAL_MOTOR_COUNT];

    /// TODO: Re-enable shell functions
    /***
    static DECL_SHELL_CMD(cmdInfo);
    static DECL_SHELL_CMD(cmdEnableFeedback);
    static DECL_SHELL_CMD(cmdPID);
    static DECL_SHELL_CMD(cmdEnableMotor);
    static DECL_SHELL_CMD(cmdEchoRaw);


#ifdef PARAM_ADJUST
    friend class PAUserGimbalThread;
#endif
    ***/

};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H

/** @} */