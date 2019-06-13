//
// Created by liuzikai on 2019-01-05.
//

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
 */
class GimbalSKD : public GimbalBase, public PIDControllerBase {

public:

    enum mode_t {
        STOP_MODE,                // zero force
        // RELATIVE_ANGLE_MODE,   // target_angle of yaw is relative to chassis
        ABS_ANGLE_MODE,           // target_angle of yaw is relative to ground
        PARAM_ADJUST_MODE         // for PID parameter adjustment program
    };

    /**
     * Start this scheduler
     * @param gimbal_ahrs_           pointer to an initialized AHRS on gimbal
     * @param gimbal_ahrs_install_   rotation matrix for gimbal AHRS
     * @param thread_prio            priority of PID calculating thread
     */
    static void start(AbstractAHRS *gimbal_ahrs_, const Matrix33 gimbal_ahrs_install_, tprio_t thread_prio);

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
     * Set target angles
     * @param yaw_target_angle    yaw   target angle on ground coordinate [deg]
     * @param pitch_target_angle  pitch target angle on ground coordinate [deg]
     */
    static void set_target_angle(float yaw_target_angle, float pitch_target_angle);


private:

    static AbstractAHRS *gimbal_ahrs;
    static Matrix33 gimbal_ahrs_install;

    // local storage
    static mode_t mode;
    static float target_angle[2];

    static float target_velocity[2];  // calculated target velocity, middle values
    static int target_current[2];     // local storage

    static PIDController a2v_pid[2];
    static PIDController v2i_pid[2];


    static constexpr unsigned int SKD_THREAD_INTERVAL = 1; // PID calculation interval [ms]

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skdThread;


    friend void cmd_gimbal_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void cmd_gimbal_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]);
    friend class GimbalDebugThread;
    friend void _cmd_gimbal_clear_i_out();

};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H
