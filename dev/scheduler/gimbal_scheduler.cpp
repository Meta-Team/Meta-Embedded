//
// Created by liuzikai on 2019-01-05.
//

#include "gimbal_scheduler.h"

Matrix33 GimbalSKD::gimbal_ahrs_install;
GimbalSKD::mode_t GimbalSKD::mode = FORCED_RELAX_MODE;
GimbalSKD::install_direction_t GimbalSKD::yaw_install;
GimbalSKD::install_direction_t GimbalSKD::pitch_install;
float GimbalSKD::target_angle[2] = {0, 0};
float GimbalSKD::target_velocity[2] = {0, 0};
int GimbalSKD::target_current[2] = {0, 0};
PIDController GimbalSKD::a2v_pid[2];
PIDController GimbalSKD::v2i_pid[2];
GimbalSKD::SKDThread GimbalSKD::skdThread;
AbstractAHRS *GimbalSKD::gimbal_ahrs = nullptr;

void GimbalSKD::start(AbstractAHRS *gimbal_ahrs_, const Matrix33 gimbal_ahrs_install_,
                      GimbalSKD::install_direction_t yaw_install_, GimbalSKD::install_direction_t pitch_install_,
                      tprio_t thread_prio) {
    gimbal_ahrs = gimbal_ahrs_;
    yaw_install = yaw_install_;
    pitch_install = pitch_install_;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            gimbal_ahrs_install[i][j] = gimbal_ahrs_install_[i][j];
        }
    }

    // Initialize accumulated angle
    Vector3D gimbal_angle = gimbal_ahrs->angle * gimbal_ahrs_install;
    accumulated_angle[YAW] = gimbal_angle.x;
    accumulated_angle[PITCH] = gimbal_angle.y;

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

GimbalSKD::mode_t GimbalSKD::get_mode() {
    return mode;
}

void GimbalSKD::set_target_angle(float yaw_target_angle, float pitch_target_angle) {
    target_angle[YAW] = yaw_target_angle;
    target_angle[PITCH] = pitch_target_angle;
}

void GimbalSKD::SKDThread::main() {
    setName("gimbal_skd");
    while (!shouldTerminate()) {

        // Fetch data
        Vector3D gimbal_angle = gimbal_ahrs->angle * gimbal_ahrs_install;
        Vector3D gimbal_gyro = gimbal_ahrs->gyro * gimbal_ahrs_install;

        float angle_movement[2] = {gimbal_angle.x - last_angle[YAW],
                                   gimbal_angle.y - last_angle[PITCH]};
        for (int i = YAW; i <= PITCH; i++) {

            /**
             * Deal with cases crossing 0 point
             * For example,
             * 1. new = 179, last = -179, movement = 358, should be corrected to 358 - 360 = -2
             * 2. new = -179, last = 179, movement = -358, should be corrected to -358 + 360 = 2
             * 200 (-200) is a threshold that is large enough that it's normally impossible to move in 1 ms
             */
            if (angle_movement[i] < -200) angle_movement[i] += 360;
            if (angle_movement[i] > 200) angle_movement[i] -= 360;

            // Use increment to calculate accumulated angles
            accumulated_angle[i] += angle_movement[i];
        }

        /* if (mode == RELATIVE_ANGLE_MODE) {

            // YAW
            target_velocity[YAW] = a2v_pid[YAW].calc(GimbalIF::feedback[YAW].actual_angle * yaw_install, target_angle[YAW]);
            // TODO: minus Chassis_MPU_w velocity from gimbal_gyro.x, or use yaw motor velocity
            target_current[YAW] = (int) v2i_pid[YAW].calc(gimbal_gyro.x, target_velocity[YAW]);

            // PITCH
            target_velocity[PITCH] = a2v_pid[PITCH].calc(GimbalIF::feedback[PITCH].actual_angle * pitch_install, target_angle[PITCH]);
            target_current[PITCH] = (int) v2i_pid[PITCH].calc(gimbal_gyro.y, target_velocity[PITCH]);

        } else */ if (mode == ABS_ANGLE_MODE) {

            // YAW
            target_velocity[YAW] = a2v_pid[YAW].calc(accumulated_angle[YAW], target_angle[YAW]);
            target_current[YAW] = (int) v2i_pid[YAW].calc(gimbal_gyro.x, target_velocity[YAW]);

            // PITCH
            target_velocity[PITCH] = a2v_pid[PITCH].calc(GimbalIF::feedback[PITCH].actual_angle * pitch_install,
                                                         target_angle[PITCH]);
            target_current[PITCH] = (int) v2i_pid[PITCH].calc(gimbal_gyro.y, target_velocity[PITCH]);

        } else if (mode == PARAM_ADJUST_MODE) {

            // TODO: write code for PID parameter adjustment mode

        } else if (mode == FORCED_RELAX_MODE) {

            target_current[YAW] = target_current[PITCH] = 0;

        }

        // Send currents
        GimbalIF::target_current[YAW] = target_current[YAW] * yaw_install;
        GimbalIF::target_current[PITCH] = target_current[PITCH] * pitch_install;
        GimbalIF::send_gimbal_currents();

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}

float GimbalSKD::get_accumulated_angle(GimbalBase::motor_id_t motor) {
    if (motor == YAW) return accumulated_angle[YAW];
    if (motor == PITCH) return accumulated_angle[PITCH];
    return 0;
}