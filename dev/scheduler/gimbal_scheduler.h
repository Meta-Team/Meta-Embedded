//
// Created by liuzikai on 2019-01-05.
//

/**
 * @file    gimbal_scheduler.h
 * @brief   Scheduler to control gimbal to meet the target, including a thread to invoke PID calculation in period.
 *
 * @addtogroup gimbal
 * @{
 */

#ifndef META_INFANTRY_GIMBAL_CONTROLLER_H
#define META_INFANTRY_GIMBAL_CONTROLLER_H

#include "ch.hpp"

#include "gimbal_interface.h"
#include "chassis_interface.h"
#include "ahrs_abstract.h"

#include "pid_controller.hpp"


/**
 * @name GimbalSKD
 * @note SKD stands for "scheduler"
 * @brief Scheduler to control gimbal to meet the target, including a thread to invoke PID calculation in period.
 * @pre GimbalIF and ChassisIF have been initialized properly
 * @usage 1. start(), load_pid_params()
 *        2. set_mode() and set_target_angle() as needed
 *        3. Leave the rest of the work to this SKD
 * @note ABOUT COORDINATE
 *       All components in this SKD use gimbal coordinate, including targets and PID controllers. Only when reading
 *       from or writing to GimbalIF will coordinate transform (by multiple to install_direction_t) be performed based
 *       on yaw_install and pitch_install
 */
class GimbalSKD : public GimbalBase, public PIDControllerBase {

public:

    enum mode_t {
        FORCED_RELAX_MODE,   // zero force (Still taking control of ChassisIF. External writing to target currents
                             // will leads to conflicts.)
        ABS_ANGLE_MODE,      // target_angle of yaw is relative to ground
        PARAM_ADJUST_MODE    // for PID parameter adjustment program
    }; // no support for RELATIVE_ANGLE_MODE

    enum install_direction_t {
        POSITIVE = 1,
        NEGATIVE = -1
    };

    /**
     * Start this scheduler
     * @param gimbal_ahrs_           Pointer to an initialized AHRS on gimbal
     * @param gimbal_ahrs_install_   Rotation matrix for gimbal AHRS
     * @param yaw_install_           Yaw motor install direction
     * @param pitch_install_         Pitch motor install direction
     * @param thread_prio            Priority of PID calculating thread
     */
    static void start(AbstractAHRS *gimbal_ahrs_, const Matrix33 gimbal_ahrs_install_,
                      install_direction_t yaw_install_, install_direction_t pitch_install_,
                      tprio_t thread_prio);

    /**
     * Set PID parameters of yaw and pitch
     * @param yaw_a2v_params
     * @param yaw_v2i_params
     * @param pitch_a2v_params
     * @param pitch_v2i_params
     */
    static void load_pid_params(pid_params_t yaw_a2v_params, pid_params_t yaw_v2i_params,
                                pid_params_t pitch_a2v_params, pid_params_t pitch_v2i_params);


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
     * @param yaw_target_angle    Yaw target ACCUMULATED angle on ground coordinate [degree]
     * @param pitch_target_angle  Pitch target ACCUMULATED angle on ground coordinate [degree]
     */
    static void set_target_angle(float yaw_target_angle, float pitch_target_angle);

    /**
     * Get accumulated angle maintained by this SKD
     * @param motor   YAW or PITCH
     * @return Accumulated angle
     */
    static float get_accumulated_angle(motor_id_t motor);

private:

    static AbstractAHRS *gimbal_ahrs;
    static Matrix33 gimbal_ahrs_install;
    static install_direction_t yaw_install;
    static install_direction_t pitch_install;

    // local storage
    static mode_t mode;
    static float target_angle[2];

    static float last_angle[2];  // last angle data of yaw and pitch from AHRS
    static float accumulated_angle[2];  // accumulated angle of yaw and pitch, since the start of this SKD

    static float target_velocity[2];  // calculated target velocity, middle values
    static int target_current[2];     // local storage

    static PIDController a2v_pid[2];
    static PIDController v2i_pid[2];


    static constexpr unsigned int SKD_THREAD_INTERVAL = 1; // PID calculation interval [ms]

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skdThread;
};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H

/** @} */