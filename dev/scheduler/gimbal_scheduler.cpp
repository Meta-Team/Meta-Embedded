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
#include "math.h"

Matrix33 GimbalSKD::ahrs_angle_rotation;
Matrix33 GimbalSKD::ahrs_gyro_rotation;
GimbalSKD::mode_t GimbalSKD::mode = FORCED_RELAX_MODE;
GimbalSKD::install_direction_t GimbalSKD::yaw_install;
GimbalSKD::install_direction_t GimbalSKD::pitch_install;
float GimbalSKD::yaw_deceleration_ratio;
float GimbalSKD::pitch_deceleration_ratio;

float GimbalSKD::target_angle[2] = {0, 0};
float GimbalSKD::target_velocity[2] = {0, 0};
int GimbalSKD::target_current[2] = {0, 0};
float GimbalSKD::last_angle[2] = {0, 0};
float GimbalSKD::accumulated_angle[2] = {0, 0};
PIDController GimbalSKD::a2v_pid[2];
PIDController GimbalSKD::v2i_pid[2];
GimbalSKD::SKDThread GimbalSKD::skdThread;
AbstractAHRS *GimbalSKD::gimbal_ahrs = nullptr;
float GimbalSKD::yaw_restrict_angle[2] = {-MAXFLOAT, MAXFLOAT};
float GimbalSKD::yaw_restrict_velocity = 0;
float GimbalSKD::pitch_restrict_angle[2] = {-MAXFLOAT, MAXFLOAT};

void
GimbalSKD::start(AbstractAHRS *gimbal_ahrs_, const Matrix33 ahrs_angle_rotation_, const Matrix33 ahrs_gyro_rotation_,
                 install_direction_t yaw_install_, install_direction_t pitch_install_, tprio_t thread_prio,
                 float yaw_deceleration_ratio_, float pitch_deceleration_ratio_) {

    gimbal_ahrs = gimbal_ahrs_;
    yaw_install = yaw_install_;
    pitch_install = pitch_install_;
    yaw_deceleration_ratio = yaw_deceleration_ratio_;
    pitch_deceleration_ratio = pitch_deceleration_ratio_;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ahrs_angle_rotation[i][j] = ahrs_angle_rotation_[i][j];
            ahrs_gyro_rotation[i][j] = ahrs_gyro_rotation_[i][j];
        }
    }

    // Initialize last_angle, to use current pointing direction as startup direction
    Vector3D ahrs_angle = gimbal_ahrs->get_angle() * ahrs_angle_rotation;

    // FIXME: find a more elegant way to handle this 
#if defined(SENTRY) || defined(AERIAL)
    last_angle[YAW] = GimbalIF::feedback[GimbalIF::YAW].actual_angle * yaw_install;
#else
    last_angle[YAW] = ahrs_angle.x - GimbalIF::feedback[GimbalIF::YAW].actual_angle * yaw_install;
    // For initial moment, angle_movement = ahrs_angle.x - last_angle[YAW] = GimbalIF::feedback[YAW].actual_angle
#endif

    last_angle[PITCH] = ahrs_angle.y;

    skdThread.start(thread_prio);
}

void GimbalSKD::load_pid_params(pid_params_t yaw_a2v_params, pid_params_t yaw_v2i_params,
                                pid_params_t pitch_a2v_params, pid_params_t pitch_v2i_params) {
    a2v_pid[YAW].change_parameters(yaw_a2v_params);
    v2i_pid[YAW].change_parameters(yaw_v2i_params);

    a2v_pid[PITCH].change_parameters(pitch_a2v_params);
    v2i_pid[PITCH].change_parameters(pitch_v2i_params);
}

void GimbalSKD::set_yaw_restriction(float yaw_min, float yaw_max, float restrict_velocity) {
    yaw_restrict_angle[0] = yaw_min;
    yaw_restrict_angle[1] = yaw_max;
    yaw_restrict_velocity = restrict_velocity;
}

void GimbalSKD::set_pitch_restriction(float pitch_min, float pitch_max) {
    pitch_restrict_angle[0] = pitch_min;
    pitch_restrict_angle[1] = pitch_max;
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

float GimbalSKD::get_target_angle(GimbalBase::motor_id_t motor) {
    return target_angle[motor];
}

void GimbalSKD::set_target_velocity(float yaw_target_velocity, float pitch_target_velocity) {
    if(GimbalSKD::get_mode() == GimbalSKD::VELOCITY_MODE){
        target_velocity[YAW] = yaw_target_velocity;
        target_velocity[PITCH] = pitch_target_velocity;
    } else {
        Shell::printf("set_target_velocity is available only when the velocity mode is enabled.");
        return;     // set_target_velocity should enable the velocity mode.
    }
}

float GimbalSKD::get_accumulated_angle(motor_id_t motor) {
    if (motor == YAW) {
        if (mode == SENTRY_MODE) {
            return GimbalIF::feedback[YAW].accumulated_angle() * yaw_install / yaw_deceleration_ratio;
        } else {
            return accumulated_angle[YAW];
        }
    } else if (motor == PITCH) {
        if (mode == SENTRY_MODE) {
            return GimbalIF::feedback[PITCH].accumulated_angle() * pitch_install / pitch_deceleration_ratio;
        } else {
            return (gimbal_ahrs->get_angle() * ahrs_angle_rotation).y;
        }
    }
    return 0;
}

float GimbalSKD::get_relative_angle(GimbalBase::motor_id_t motor) {
    if (motor == YAW) {
        return GimbalIF::feedback[YAW].accumulated_angle() * yaw_install / yaw_deceleration_ratio;
    } else if (motor == PITCH) {
        return GimbalIF::feedback[PITCH].accumulated_angle() * pitch_install / pitch_deceleration_ratio;
    }
    return 0;
}

float GimbalSKD::get_actual_velocity(motor_id_t motor) {
    Vector3D ahrs_angle = gimbal_ahrs->get_angle() * ahrs_angle_rotation;
    Vector3D ahrs_gyro = gimbal_ahrs->get_gyro() * ahrs_gyro_rotation;
    float velocity[2] = {
            ahrs_gyro.x * cosf(ahrs_angle.y / 180.0f * M_PI) + ahrs_gyro.z * sinf(ahrs_angle.y / 180.0f * M_PI),
            ahrs_gyro.y};
    if (motor == YAW) {
        return velocity[0];
    } else if (motor == PITCH) {
        return velocity[1];
    }
    return 0;
}

float GimbalSKD::get_target_velocity(motor_id_t motor) {
    if (motor == YAW) {
        return target_velocity[YAW];
    } else if (motor == PITCH) {
        return target_velocity[PITCH];
    }
    return 0;
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
            // Use AHRS angle and velocity

            target_velocity[YAW] = a2v_pid[YAW].calc(accumulated_angle[YAW], target_angle[YAW]);
            // Perform crop on physical relative angle of Yaw
            if (GimbalIF::feedback[YAW].accumulated_angle() * yaw_install > yaw_restrict_angle[1] &&
                target_velocity[YAW] > yaw_restrict_velocity) {

                target_velocity[YAW] = yaw_restrict_velocity;

            } else if (GimbalIF::feedback[YAW].accumulated_angle() * yaw_install < yaw_restrict_angle[0] &&
                       target_velocity[YAW] < -yaw_restrict_velocity) {

                target_velocity[YAW] = -yaw_restrict_velocity;

            }
            target_current[YAW] = (int) v2i_pid[YAW].calc(velocity[YAW], target_velocity[YAW]);


            /// Pitch
            // Use AHRS angle and AHRS velocity
            target_velocity[PITCH] = a2v_pid[PITCH].calc(angle[PITCH], target_angle[PITCH]);
            target_current[PITCH] = (int) v2i_pid[PITCH].calc(velocity[PITCH], target_velocity[PITCH]);

        } else if (mode == SENTRY_MODE) {

            /// Yaw
            // Use gimbal motor feedback angle and velocity
            target_velocity[YAW] = a2v_pid[YAW].calc(
                    GimbalIF::feedback[YAW].accumulated_angle() * yaw_install / yaw_deceleration_ratio,
                    target_angle[YAW]);
            target_current[YAW] = (int) v2i_pid[YAW].calc(
                    GimbalIF::feedback[YAW].actual_velocity * yaw_install / yaw_deceleration_ratio,
                    target_velocity[YAW]);

            /// Pitch
            // Use gimbal motor feedback angle and AHRS velocity
            target_velocity[PITCH] = a2v_pid[PITCH].calc(
                    GimbalIF::feedback[PITCH].accumulated_angle() * pitch_install / pitch_deceleration_ratio,
                    target_angle[PITCH]);
            target_current[PITCH] = (int) v2i_pid[PITCH].calc(velocity[PITCH], target_velocity[PITCH]);

        }else if (mode == VELOCITY_MODE){

            /// Yaw
            if (GimbalIF::feedback[YAW].accumulated_angle() * yaw_install < yaw_restrict_angle[1] &&
                    GimbalIF::feedback[YAW].accumulated_angle() * yaw_install > yaw_restrict_angle[0] ) {

                target_current[YAW] = (int) v2i_pid[YAW].calc(velocity[YAW], target_velocity[YAW]);

            } else {
                target_current[YAW] = 0;
            }

            /// Pitch
            float pitch_angle = angle[PITCH];
            if (pitch_angle > pitch_restrict_angle[0] && pitch_angle < pitch_restrict_angle[1]) {
                target_current[PITCH] = (int) v2i_pid[PITCH].calc(velocity[PITCH], target_velocity[PITCH]);
            } else {
                target_current[PITCH] = 0;
            }

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



/** @} */