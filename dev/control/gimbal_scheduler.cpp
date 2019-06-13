//
// Created by liuzikai on 2019-01-05.
//

#include "gimbal_scheduler.h"

Matrix33 GimbalSKD::gimbal_ahrs_install;
GimbalSKD::mode_t GimbalSKD::mode = STOP_MODE;
float GimbalSKD::target_angle[2] = {0, 0};
float GimbalSKD::target_velocity[2] = {0, 0};
int GimbalSKD::target_current[2] = {0, 0};
PIDController GimbalSKD::a2v_pid[2];
PIDController GimbalSKD::v2i_pid[2];
GimbalSKD::SKDThread GimbalSKD::skdThread;
AbstractAHRS* GimbalSKD::gimbal_ahrs = nullptr;

void GimbalSKD::start(AbstractAHRS* gimbal_ahrs_, const Matrix33 gimbal_ahrs_install_, tprio_t thread_prio) {
    gimbal_ahrs = gimbal_ahrs_;

    for (int i = 0 ; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            gimbal_ahrs_install[i][j] = gimbal_ahrs_install_[i][j];
        }
    }

    skdThread.start(thread_prio);
}

void GimbalSKD::load_pid_params(pid_params_t yaw_a2v_params, pid_params_t yaw_v2i_params,
                                pid_params_t pitch_a2v_params, pid_params_t pitch_v2i_params) {
    a2v_pid[YAW].change_parameters(yaw_a2v_params);
    v2i_pid[YAW].change_parameters(yaw_v2i_params);

    a2v_pid[PITCH].change_parameters(pitch_a2v_params);
    v2i_pid[PITCH].change_parameters(pitch_v2i_params);
}

void GimbalSKD::set_mode(GimbalSKD::mode_t skd_mode) {
    mode = skd_mode;
}

void GimbalSKD::set_target_angle(float yaw_target_angle, float pitch_target_angle) {
    target_angle[YAW] = yaw_target_angle;
    target_angle[PITCH] = pitch_target_angle;
}

void GimbalSKD::SKDThread::main() {
    setName("gimbal_skd");
    while(!shouldTerminate()) {

        // Fetch data
        Vector3D gimbal_angle = gimbal_ahrs->angle * gimbal_ahrs_install;
        Vector3D gimbal_gyro = gimbal_ahrs->gyro * gimbal_ahrs_install;

        /* if (mode == RELATIVE_ANGLE_MODE) {

            // YAW
            target_velocity[YAW] = a2v_pid[YAW].calc(GimbalIF::feedback[YAW].actual_angle, target_angle[YAW]);
            // TODO: minus Chassis_MPU_w velocity from gimbal_gyro.x, or use yaw motor velocity
            target_current[YAW] = (int) v2i_pid[YAW].calc(gimbal_gyro.x, target_velocity[YAW]);

            // PITCH
            target_velocity[PITCH] = a2v_pid[PITCH].calc(GimbalIF::feedback[PITCH].actual_angle, target_angle[PITCH]);
            target_current[PITCH] = (int) v2i_pid[PITCH].calc(gimbal_gyro.y, target_velocity[PITCH]);

        } else */ if (mode == ABS_ANGLE_MODE) {

            // YAW
            target_velocity[YAW] = a2v_pid[YAW].calc(gimbal_angle.x, target_angle[YAW]);
            target_current[YAW] = (int) v2i_pid[YAW].calc(gimbal_gyro.x, target_velocity[YAW]);

            // PITCH
            target_velocity[PITCH] = a2v_pid[PITCH].calc(GimbalIF::feedback[PITCH].actual_angle, target_angle[PITCH]);
            target_current[PITCH] = (int) v2i_pid[PITCH].calc(gimbal_gyro.y, target_velocity[PITCH]);

        } else if (mode == PARAM_ADJUST_MODE) {

            // TODO: write code for PID parameter adjustment mode

        } else if (mode == STOP_MODE) {

            target_current[YAW] = target_current[PITCH] = 0;

        }

        // Send currents
        GimbalIF::target_current[YAW] = target_current[YAW];
        GimbalIF::target_current[PITCH] = target_current[PITCH];
        GimbalIF::send_gimbal_currents();

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}