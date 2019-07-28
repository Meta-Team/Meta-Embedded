//
// Created by liuzikai on 2019-01-05.
//

/**
 * @file    gimbal_scheduler.cpp
 * @brief   Scheduler to control gimbal to meet the target, including a thread to invoke PID calculation in period.
 *
 * @addtogroup gimbal
 * @{
 */

#include "gimbal_scheduler.h"

Matrix33 GimbalSKD::ahrs_angle_rotation;
Matrix33 GimbalSKD::ahrs_gyro_rotation;
GimbalSKD::mode_t GimbalSKD::mode = FORCED_RELAX_MODE;
GimbalSKD::install_direction_t GimbalSKD::yaw_install;
GimbalSKD::install_direction_t GimbalSKD::pitch_install;
float GimbalSKD::target_angle[2] = {0, 0};
float GimbalSKD::target_velocity[2] = {0, 0};
int GimbalSKD::target_current[2] = {0, 0};
float GimbalSKD::last_angle[2] = {0, 0};
float GimbalSKD::accumulated_angle[2] = {0, 0};
PIDController GimbalSKD::a2v_pid[2];
PIDController GimbalSKD::v2i_pid[2];
GimbalSKD::SKDThread GimbalSKD::skdThread;
AbstractAHRS *GimbalSKD::gimbal_ahrs = nullptr;

void
GimbalSKD::start(AbstractAHRS *gimbal_ahrs_, const Matrix33 ahrs_angle_rotation_, const Matrix33 ahrs_gyro_rotation_,
                 install_direction_t yaw_install_, install_direction_t pitch_install_, tprio_t thread_prio) {
    gimbal_ahrs = gimbal_ahrs_;
    yaw_install = yaw_install_;
    pitch_install = pitch_install_;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ahrs_angle_rotation[i][j] = ahrs_angle_rotation_[i][j];
            ahrs_gyro_rotation[i][j] = ahrs_gyro_rotation_[i][j];
        }
    }

#if !defined(SENTRY) && !defined(AERIAL)
    // Initialize last_angle, to use current pointing direction as startup direction
    Vector3D ahrs_angle = gimbal_ahrs->get_angle() * ahrs_angle_rotation;

    last_angle[YAW] = ahrs_angle.x;
    last_angle[PITCH] = ahrs_angle.y;
#endif

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
    setName("GimbalSKD");
    while (!shouldTerminate()) {

        // Fetch data
        Vector3D ahrs_angle = gimbal_ahrs->get_angle() * ahrs_angle_rotation;
        Vector3D ahrs_gyro = gimbal_ahrs->get_gyro() * ahrs_gyro_rotation;

        // TODO: document calculations here
        float angle[2] = {ahrs_angle.x, ahrs_angle.y};
        float velocity[2] = {
                ahrs_gyro.x * cosf(ahrs_angle.y / 180.0f * M_PI) + ahrs_gyro.z * sinf(ahrs_angle.y / 180.0f * M_PI),
                ahrs_gyro.y};

        for (int i = YAW; i <= PITCH; i++) {

            float angle_movement = angle[i] - last_angle[i];
            last_angle[i] = angle[i];

            /**
             * Deal with cases crossing 0 point
             * For example,
             * 1. new = 179, last = -179, movement = 358, should be corrected to 358 - 360 = -2
             * 2. new = -179, last = 179, movement = -358, should be corrected to -358 + 360 = 2
             * 200 (-200) is a threshold that is large enough that it's normally impossible to move in 1 ms
             */
            if (angle_movement < -200) angle_movement += 360;
            if (angle_movement > 200) angle_movement -= 360;

            // Use increment to calculate accumulated angles
            accumulated_angle[i] += angle_movement;
        }

        if (mode == ABS_ANGLE_MODE) {

            /// Yaw
#if defined(SENTRY) || defined(AERIAL)
            // Use gimbal feedback angle and velocity
            target_velocity[YAW] = a2v_pid[YAW].calc(GimbalIF::feedback[YAW].accumulated_angle() * yaw_install, target_angle[YAW]);
            target_current[YAW] = (int) v2i_pid[YAW].calc(GimbalIF::feedback[YAW].actual_velocity * yaw_install, target_velocity[YAW]);
#else
            // Use AHRS angle and velocity
            target_velocity[YAW] = a2v_pid[YAW].calc(accumulated_angle[YAW], target_angle[YAW]);
            target_current[YAW] = (int) v2i_pid[YAW].calc(velocity[YAW], target_velocity[YAW]);
#endif

            /// Pitch

#if defined(SENTRY) || defined(AERIAL)
            // Use gimbal feedback angle and AHRS velocity
            target_velocity[PITCH] = a2v_pid[PITCH].calc(GimbalIF::feedback[PITCH].accumulated_angle() * pitch_install, target_angle[PITCH]);
#else
            // Use AHRS feedback angle and AHRS velocity
            target_velocity[PITCH] = a2v_pid[PITCH].calc(angle[PITCH], target_angle[PITCH]);
#endif

            target_current[PITCH] = (int) v2i_pid[PITCH].calc(velocity[PITCH], target_velocity[PITCH]);

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
    if (motor == YAW) {
#if defined(SENTRY) || defined(AERIAL)
        return GimbalIF::feedback[YAW].accumulated_angle() * yaw_install;
#else
        return accumulated_angle[YAW];
#endif
    }
    if (motor == PITCH) return (gimbal_ahrs->get_angle() * ahrs_angle_rotation).y;
    return 0;
}

/** @} */