//
// Created by zhukerui on 2019/6/8.
//

#include "suspension_gimbal_skd.h"

SuspensionGimbalSKD::mode_t SuspensionGimbalSKD::mode = STOP_MODE;
float SuspensionGimbalSKD::target_angle[2] = {0, 0};
float SuspensionGimbalSKD::target_velocity[2] = {0, 0};
int SuspensionGimbalSKD::target_current[2] = {0, 0};
PIDController SuspensionGimbalSKD::a2v_pid[2];
PIDController SuspensionGimbalSKD::v2i_pid[2];
SuspensionGimbalSKD::SKDThread SuspensionGimbalSKD::skdThread;

void SuspensionGimbalSKD::start(tprio_t thread_prio) {
    skdThread.start(thread_prio);
}

void SuspensionGimbalSKD::load_pid_params(pid_params_t yaw_a2v_params, pid_params_t yaw_v2i_params,
                                pid_params_t pitch_a2v_params, pid_params_t pitch_v2i_params) {
    a2v_pid[YAW].change_parameters(yaw_a2v_params);
    v2i_pid[YAW].change_parameters(yaw_v2i_params);

    a2v_pid[PITCH].change_parameters(pitch_a2v_params);
    v2i_pid[PITCH].change_parameters(pitch_v2i_params);
}

void SuspensionGimbalSKD::set_mode(SuspensionGimbalSKD::mode_t skd_mode) {
    mode = skd_mode;
}

void SuspensionGimbalSKD::set_target_angle(float yaw_target_angle, float pitch_target_angle) {
    target_angle[YAW] = yaw_target_angle;
    target_angle[PITCH] = pitch_target_angle;
}

void SuspensionGimbalSKD::SKDThread::main() {
    setName("gimbal_skd");
    while(!shouldTerminate()) {

        if (mode == RELATIVE_ANGLE_MODE) {

            // YAW
            target_velocity[YAW] = a2v_pid[YAW].calc(GimbalIF::feedback[YAW].actual_angle, target_angle[YAW]);
            target_current[YAW] = (int) v2i_pid[YAW].calc(Gimbal_AHRS_Yaw_W - Chassis_MPU_W, target_velocity[YAW]);

            // PITCH
            target_velocity[PITCH] = a2v_pid[PITCH].calc(GimbalIF::feedback[PITCH].actual_angle, target_angle[PITCH]);
            target_current[PITCH] = (int) v2i_pid[PITCH].calc(Gimbal_AHRS_Pitch_Angle, target_velocity[PITCH]);

        } else if (mode == ABS_ANGLE_MODE) {

            // YAW
            target_velocity[YAW] = a2v_pid[YAW].calc(Gimbal_AHRS_Yaw_Angle, target_angle[YAW]) - Chassis_MPU_W;
            target_current[YAW] = (int) v2i_pid[YAW].calc(GimbalIF::feedback[YAW].actual_velocity, target_velocity[YAW]);

            // PITCH
            target_velocity[PITCH] = a2v_pid[PITCH].calc(GimbalIF::feedback[PITCH].actual_angle, target_angle[PITCH]);
            target_current[PITCH] = (int) v2i_pid[PITCH].calc(Gimbal_AHRS_Pitch_Angle, target_velocity[PITCH]);

        } else if (mode == PARAM_ADJUST_MODE) {

            // TODO: write code for PID parameter adjustment mode

        } else if (mode == STOP_MODE) {

            target_current[YAW] = target_current[PITCH] = 0;

        }

        // Send Currents
        GimbalIF::target_current[YAW] = target_current[YAW];
        GimbalIF::target_current[PITCH] = target_current[PITCH];
        GimbalIF::send_gimbal_currents();

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}