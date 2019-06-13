//
// Created by zhukerui on 2019/6/8.
//

#ifndef META_INFANTRY_SUSPENSION_GIMBAL_CALCULATOR_H
#define META_INFANTRY_SUSPENSION_GIMBAL_CALCULATOR_H

#include "suspension_gimbal_interface.h"
#include "suspension_gimbal_skd.h"
#include "chassis_interface.h"
#include "pid_controller.h"

/**
 * @name SuspensionGimbalSKD
 * @note SKD stands for "scheduler"
 * @brief A thread to invoke PID calculation in period.
 */
class SuspensionGimbalSKD : public GimbalBase, public PIDController {

public:

    enum mode_t {
        STOP_MODE,             // zero force
        RELATIVE_ANGLE_MODE,   // target_angle of yaw is relative to chassis
        ABS_ANGLE_MODE,        // target_angle of yaw is relative to ground
        PARAM_ADJUST_MODE      // for PID parameter adjustment program
    };

    static void start(tprio_t thread_prio);

    /**
     * Set PID parameters of yaw and pitch
     * @param yaw_a2v_params
     * @param yaw_v2i_params
     * @param pitch_a2v_params
     * @param pitch_v2i_params
     */
    static void load_pid_params(pid_params_t yaw_a2v_params, pid_params_t yaw_v2i_params,
                                  pid_params_t pitch_a2v_params, pid_params_t pitch_v2i_params);


    static void set_mode(mode_t skd_mode);

    static void set_target_angle(float yaw_target_angle, float pitch_target_angle);


private:

    static constexpr unsigned int SKD_THREAD_INTERVAL = 1; // PID calculation interval [ms]

    static mode_t mode;

    static float target_angle[2];

    /**
     * @brief calculated target velocity
     * @note middle value. For outside code, only for test.
     */
    static float target_velocity[2];

    static int target_current[2];

    static PIDController a2v_pid[2];
    static PIDController v2i_pid[2];

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skdThread;



    friend void cmd_gimbal_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void cmd_gimbal_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]);
    friend class GimbalDebugThread;
    friend void _cmd_gimbal_clear_i_out();

};

#endif //META_INFANTRY_SUSPENSION_GIMBAL_CALCULATOR_H
