//
// Created by liuzikai on 2019-01-05.
//

/**
 * @file    gimbal_scheduler.h
 * @brief   Scheduler to control gimbal to meet the target, including a thread to invoke PID calculation in period.
 *
 * @version 3.0a
 * @date 2.28, 2022
 * @addtogroup gimbal
 * @{
 */

#ifndef META_INFANTRY_GIMBAL_CONTROLLER_H
#define META_INFANTRY_GIMBAL_CONTROLLER_H

#include "ch.hpp"

#include "can_motor_scheduler.h"
#include "ahrs_abstract.h"

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
 * @usage 1. start(), load_pid_params()
 *        2. set_mode() and set_target_angle() as needed
 *        3. Leave the rest of the work to this SKD
 * @note ABOUT COORDINATE
 *       All components in this SKD use GIMBAL coordinate (rather than those of motors,
 *       Yaw: CCW as positive, Pitch: Up as positive), including targets and PID controllers. Only when reading from or
 *       writing to GimbalIF will coordinate transform (by multiple to install_direction_t) be performed based on
 *       yaw_install and pitch_install
 * @note ABOUT TARGET ANGLES
 *       APIs use angles of GIMBAL (rather than those of motors). But for local storage, deceleration_ratio is included.
 *       Conversions are performed when APIs are invoked.
 */

class GimbalSKD {

public:
    enum angle_id_t{
        YAW,
        PITCH,
#if defined(ENABLE_SUBPITCH)
        SUB_PITCH,
#endif
        GIMBAL_MOTOR_COUNT
    };

    enum mode_t {
        FORCED_RELAX_MODE,  // zero force (but still taking control of GimbalIF)
        CHASSIS_REF_MODE,   //
        GIMBAL_REF_MODE // TODO: Use MIMO controller to optimize this mode
    };

    /**
     * Start this scheduler
     * @param gimbal_ahrs_                Pointer to an initialized AHRS on gimbal
     * @param ahrs_angle_rotation_        Rotation matrix for AHRS angle
     * @param ahrs_gyro_rotation_         Rotation matrix for AHRS gyro
     * @param yaw_install_                Yaw motor install direction
     * @param pitch_install_              Pitch motor install direction
     * @param sub_pitch_install_          Sub-pitch motor install direction
     * @param thread_prio                 Priority of PID calculating thread
     */
    static void start(AbstractAHRS *gimbal_ahrs_, const Matrix33 ahrs_angle_rotation_, const Matrix33 ahrs_gyro_rotation_, tprio_t thread_prio);

    /**
     * Set mode of this SKD
     * @param skd_mode
     */
    static void set_mode(mode_t skd_mode);

    /**
     * Get mode of this SKD
     * @return Current mode
     */
    static mode_t get_mode();

    /**
     * Set target angles
     * @param yaw_target_angle    GIMBAL yaw target ACCUMULATED angle on ground coordinate [degree]
     * @param pitch_target_angle  GIMBAL pitch target ACCUMULATED angle on ground coordinate [degree]
     * @param sub_pitch_target_angle  GIMBAL sub-pitch target ACCUMULATED angle on pitch coordinate [degree]
     */
    static void set_target_angle(float yaw_target_angle, float pitch_target_angle, float sub_pitch_target_angle = 0);

    /**
     * Get current target angle data
     * @param motor   YAW or PITCH or SUB_PITCH
     * @return Current target angle of GIMBAL
     */
    static float get_target_angle(GimbalSKD::angle_id_t motor);

    /// TODO: Re-enable shell functions
//    static void cmdFeedback(void *);
    static const Shell::Command shellCommands[];

private:

    static AbstractAHRS *gimbal_ahrs;
    static Matrix33 ahrs_angle_rotation;
    static Matrix33 ahrs_gyro_rotation;

    // Local storage
    static mode_t mode;

    static bool motor_enable[GIMBAL_MOTOR_COUNT];
#ifdef PARAM_ADJUST
    static bool a2v_pid_enabled;  // only allowed by PAUser
#endif

    static float target_angle[GIMBAL_MOTOR_COUNT];  // angles of MOTORS (NOTICE: different from target_angle in set_target_angle())
    static float last_angle[GIMBAL_MOTOR_COUNT];  // last angle data of yaw and pitch from AHRS
    static float accumulated_angle[GIMBAL_MOTOR_COUNT];  // accumulated angle of yaw and pitch motors, since the start of this SKD

    static float target_velocity[GIMBAL_MOTOR_COUNT];  // calculated target velocity, middle values
    static int target_current[GIMBAL_MOTOR_COUNT];     // local storage

    static float actual_velocity[GIMBAL_MOTOR_COUNT];

    static constexpr unsigned int SKD_THREAD_INTERVAL = 1; // PID calculation interval [ms]

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skd_thread;

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