//
// Created by liuzikai on 2019-01-05.
//

#ifndef META_INFANTRY_GIMBAL_CONTROLLER_H
#define META_INFANTRY_GIMBAL_CONTROLLER_H

#include "gimbal_interface.h"
#include "pid_controller.hpp"

/**
 * @name Gimbal
 * @brief Gimbal controller from high level to low level (by inheritance)
 * @note
 */
class Gimbal : public GimbalInterface, public PIDControllerBase {

public:

    /**
     * Set PID parameters of yaw and pitch
     * @param yaw_a2v_params
     * @param yaw_v2i_params
     * @param pitch_a2v_params
     * @param pitch_v2i_params
     */
    static void change_pid_params(pid_params_t yaw_a2v_params, pid_params_t yaw_v2i_params,
                                  pid_params_t pitch_a2v_params, pid_params_t pitch_v2i_params);

    /**
     * Perform calculation from angle to current and put result into target_current[]
     * @param yaw_actual_velocity
     * @param pitch_actual_velocity
     * @param yaw_target_angle
     * @param pitch_target_angle
     */
    static void calc_gimbal(float yaw_actual_velocity, float pitch_actual_velocity,
                            float yaw_target_angle, float pitch_target_angle);



private:

    /**
     * @brief calculated target velocity
     * @note middle value. For outside code, only for test.
     */
    static float target_velocity[2];

    static PIDController a2v_pid[2];
    static PIDController v2i_pid[2];

    /**
     * Perform calculation from angle to velocity and put result into target_velocity_[]
     * @param id
     * @param actual_angle
     * @param target_angle
     * @note for outside code, only for test.
     */
    static void calc_a2v_(motor_id_t id_, float actual_angle_, float target_angle_);

    /**
     * Perform calculation from velocity to current and put result into target_current[]
     * @param id
     * @param actual_velocity
     * @param target_velocity
     * @note for outside code, only for test.
     */
    static void calc_v2i_(motor_id_t id_, float actual_velocity_, float target_velocity_);

    friend void cmd_gimbal_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void cmd_gimbal_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]);
    friend class GimbalDebugThread;
    friend void _cmd_gimbal_clear_i_out();

};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H
