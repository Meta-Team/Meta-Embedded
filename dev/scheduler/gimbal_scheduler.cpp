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
GimbalSKD::install_direction_t GimbalSKD::sub_pitch_install;
float GimbalSKD::yaw_deceleration_ratio;
float GimbalSKD::pitch_deceleration_ratio;
float GimbalSKD::sub_pitch_deceleration_ratio;

bool GimbalSKD::is_test = false;
bool GimbalSKD::motor_enable[3] = {false, false, false};
float GimbalSKD::target_angle[3] = {0, 0, 0};
float GimbalSKD::target_velocity[3] = {0, 0, 0};
int GimbalSKD::target_current[3] = {0, 0, 0};
float GimbalSKD::last_angle[3] = {0, 0, 0};
float GimbalSKD::accumulated_angle[3] = {0, 0, 0};
PIDController GimbalSKD::a2v_pid[3];
PIDController GimbalSKD::v2i_pid[3];
GimbalSKD::SKDThread GimbalSKD::skd_thread;
AbstractAHRS *GimbalSKD::gimbal_ahrs = nullptr;
float GimbalSKD::yaw_restrict_angle[2] = {-MAXFLOAT, MAXFLOAT};
float GimbalSKD::yaw_restrict_velocity = 0;
//float GimbalSKD::actual_angle[3] = {0, 0, 0};
float GimbalSKD::actual_velocity[3] = {0, 0, 0};

void
GimbalSKD::start(AbstractAHRS *gimbal_ahrs_, const Matrix33 ahrs_angle_rotation_, const Matrix33 ahrs_gyro_rotation_,
                 install_direction_t yaw_install_, install_direction_t pitch_install_, install_direction_t sub_pitch_install_, tprio_t thread_prio,
                 float yaw_deceleration_ratio_, float pitch_deceleration_ratio_, float sub_pitch_deceleration_ratio_) {

    gimbal_ahrs = gimbal_ahrs_;
    yaw_install = yaw_install_;
    pitch_install = pitch_install_;
    sub_pitch_install = sub_pitch_install_;
    yaw_deceleration_ratio = yaw_deceleration_ratio_;
    pitch_deceleration_ratio = pitch_deceleration_ratio_;
    sub_pitch_deceleration_ratio = sub_pitch_deceleration_ratio_;

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
    last_angle[YAW] = GimbalIF::feedback[GimbalIF::YAW]->actual_angle * yaw_install;
#else
    last_angle[YAW] = ahrs_angle.x - GimbalIF::feedback[GimbalIF::YAW]->actual_angle * float(yaw_install);
    // For initial moment, angle_movement = ahrs_angle.x - last_angle[YAW] = GimbalIF::feedback[YAW].actual_angle
#endif

    last_angle[PITCH] = ahrs_angle.y;
    last_angle[SUB_PITCH] = GimbalIF::feedback[SUB_PITCH]->actual_angle;

    skd_thread.start(thread_prio);
}

void GimbalSKD::load_pid_params(pid_params_t yaw_a2v_params, pid_params_t yaw_v2i_params,
                                pid_params_t pitch_a2v_params, pid_params_t pitch_v2i_params,
                                pid_params_t sub_pitch_a2v_params, pid_params_t sub_pitch_v2i_params) {
    a2v_pid[YAW].change_parameters(yaw_a2v_params);
    v2i_pid[YAW].change_parameters(yaw_v2i_params);

    a2v_pid[PITCH].change_parameters(pitch_a2v_params);
    v2i_pid[PITCH].change_parameters(pitch_v2i_params);

    a2v_pid[SUB_PITCH].change_parameters(sub_pitch_a2v_params);
    v2i_pid[SUB_PITCH].change_parameters(sub_pitch_v2i_params);
}

void GimbalSKD::load_pid_params_by_type(pid_params_t pid_params, motor_id_t motor_id, bool is_a2v) {
    if (motor_id > SUB_PITCH) {
        return;
    }
    PIDController *p = is_a2v ? a2v_pid : v2i_pid;
    p[motor_id].change_parameters(pid_params);
}

PIDController::pid_params_t GimbalSKD::echo_pid_params_by_type(motor_id_t motor_id, bool is_a2v) {
    if (motor_id > SUB_PITCH) {
        return {0,0,0,0,0};
    }
    PIDController *p = is_a2v ? a2v_pid : v2i_pid;
    return p[motor_id].get_parameters();
}

void GimbalSKD::set_yaw_restriction(float yaw_min, float yaw_max, float restrict_velocity) {
    yaw_restrict_angle[0] = yaw_min;
    yaw_restrict_angle[1] = yaw_max;
    yaw_restrict_velocity = restrict_velocity;
}

void GimbalSKD::set_mode(GimbalSKD::mode_t skd_mode) {
    mode = skd_mode;
}

GimbalSKD::mode_t GimbalSKD::get_mode() {
    return mode;
}

void GimbalSKD::set_target_angle(float yaw_target_angle, float pitch_target_angle, float sub_pitch_target_angle) {
    target_angle[YAW] = yaw_target_angle;
    target_angle[PITCH] = pitch_target_angle;
    target_angle[SUB_PITCH] = sub_pitch_target_angle;
}

float GimbalSKD::get_target_angle(GimbalBase::motor_id_t motor) {
    return target_angle[motor];
}

float GimbalSKD::get_target_velocity(motor_id_t motor) {
    return target_velocity[motor];
}

void GimbalSKD::SKDThread::main() {
    setName("GimbalSKD");
    while (!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            // Fetch data
            Vector3D ahrs_angle = gimbal_ahrs->get_angle() * ahrs_angle_rotation;
            Vector3D ahrs_gyro = gimbal_ahrs->get_gyro() * ahrs_gyro_rotation;

            // TODO: document calculations here
            float angle[2] = {ahrs_angle.x, ahrs_angle.y};
            float velocity[2] = {
                    ahrs_gyro.x * cosf(ahrs_angle.y / 180.0f * M_PI) + ahrs_gyro.z * sinf(ahrs_angle.y / 180.0f * M_PI),
                    ahrs_gyro.y};


            float yaw_angle_movement = angle[0] - last_angle[YAW];
            last_angle[YAW] = angle[0];

            /**
             * Deal with cases crossing 0 point
             * For example,
             * 1. new = 179, last = -179, movement = 358, should be corrected to 358 - 360 = -2
             * 2. new = -179, last = 179, movement = -358, should be corrected to -358 + 360 = 2
             * 200 (-200) is a threshold that is large enough that it's normally impossible to move in 1 ms
             */
            if (yaw_angle_movement < -200) yaw_angle_movement += 360;
            if (yaw_angle_movement > 200) yaw_angle_movement -= 360;

            // Use increment to calculate accumulated angles
            accumulated_angle[YAW] += yaw_angle_movement;
            actual_velocity[YAW] = velocity[0];

            last_angle[PITCH] = angle[1];
            accumulated_angle[PITCH] = angle[1];
            actual_velocity[PITCH] = velocity[1];


            last_angle[SUB_PITCH] = accumulated_angle[SUB_PITCH];
            accumulated_angle[SUB_PITCH] = GimbalIF::feedback[SUB_PITCH]->actual_angle;
            actual_velocity[SUB_PITCH] = GimbalIF::feedback[SUB_PITCH]->actual_velocity;

            if (mode == ABS_ANGLE_MODE) {

                /// Yaw
                if (!is_test || (motor_enable[YAW])) {
                // Use AHRS angle and velocity
                    target_velocity[YAW] = a2v_pid[YAW].calc(accumulated_angle[YAW], target_angle[YAW]);
                    // Perform crop on physical relative angle of Yaw

#if !(defined(HERO) || defined(INFANTRY))
                    if (GimbalIF::feedback[YAW]->accumulated_angle() * yaw_install > yaw_restrict_angle[1] &&
                        target_velocity[YAW] > yaw_restrict_velocity) {

                        target_velocity[YAW] = yaw_restrict_velocity;

                    } else if (GimbalIF::feedback[YAW]->accumulated_angle() * yaw_install < yaw_restrict_angle[0] &&
                               target_velocity[YAW] < -yaw_restrict_velocity) {

                        target_velocity[YAW] = -yaw_restrict_velocity;

                    }
#endif

                    target_current[YAW] = (int) v2i_pid[YAW].calc(actual_velocity[YAW], target_velocity[YAW]);
                }

                /// Pitch
                if (!is_test || (motor_enable[PITCH])) {
                    // Use AHRS angle and AHRS velocity
                    target_velocity[PITCH] = a2v_pid[PITCH].calc(last_angle[PITCH], target_angle[PITCH]);
                    target_current[PITCH] = (int) v2i_pid[PITCH].calc(actual_velocity[PITCH], target_velocity[PITCH]);
                }
                /// Sub-pitch
                if (!is_test || (motor_enable[SUB_PITCH])) {
//                    target_velocity[SUB_PITCH] = a2v_pid[SUB_PITCH].calc(accumulated_angle[SUB_PITCH], target_angle[SUB_PITCH]);
//                    target_current[SUB_PITCH] = (int) v2i_pid[SUB_PITCH].calc(actual_velocity[SUB_PITCH], target_velocity[SUB_PITCH]);
                    target_current[SUB_PITCH] = (int) a2v_pid[SUB_PITCH].calc(accumulated_angle[SUB_PITCH], target_angle[SUB_PITCH]);
                }
            }
//            else if (mode == SENTRY_MODE) {
//
//                /// Yaw
//                // Use gimbal motor feedback angle and velocity
//                actual_angle[YAW] = GimbalIF::feedback[YAW]->accumulated_angle() * float(yaw_install) / yaw_deceleration_ratio;
//                target_velocity[YAW] = a2v_pid[YAW].calc(actual_angle[YAW], target_angle[YAW]);
//
//                actual_velocity[YAW] = GimbalIF::feedback[YAW]->actual_velocity * float(yaw_install) / yaw_deceleration_ratio;
//                target_current[YAW] = (int) v2i_pid[YAW].calc(actual_velocity[YAW], target_velocity[YAW]);
//
//                /// Pitch
//                // Use gimbal motor feedback angle and AHRS velocity
//                actual_angle[PITCH] =
//                        GimbalIF::feedback[PITCH]->accumulated_angle() * float(pitch_install) / pitch_deceleration_ratio;
//                target_velocity[PITCH] = a2v_pid[PITCH].calc(actual_angle[PITCH], target_angle[PITCH]);
//
//                actual_velocity[PITCH] = velocity[PITCH];
//                target_current[PITCH] = (int) v2i_pid[PITCH].calc(actual_velocity[PITCH], target_velocity[PITCH]);
//
//            }
            else if (mode == FORCED_RELAX_MODE) {
                target_current[YAW] = target_current[PITCH] = target_current[SUB_PITCH] = 0;
            }

            if (is_test) {
                for (int i = YAW; i <= SUB_PITCH; i++) {
                    target_current[i] = motor_enable[i] ? target_current[i] : 0;
                }
            }
            // Send currents
            *GimbalIF::target_current[YAW] = target_current[YAW] * yaw_install;
            *GimbalIF::target_current[PITCH] = target_current[PITCH] * pitch_install;
            *GimbalIF::target_current[SUB_PITCH] = target_current[SUB_PITCH] * sub_pitch_install;
            GimbalIF::clip_gimbal_current();
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}

float GimbalSKD::get_accumulated_angle(motor_id_t motor) {
//    if (motor == YAW) {
//        if (mode == SENTRY_MODE) {
//            return GimbalIF::feedback[YAW]->accumulated_angle() * float(yaw_install) / yaw_deceleration_ratio;
//        } else {
//            return accumulated_angle[YAW];
//        }
//    } else if (motor == PITCH) {
//        if (mode == SENTRY_MODE) {
//            return GimbalIF::feedback[PITCH]->accumulated_angle() * float(pitch_install) / pitch_deceleration_ratio;
//        } else {
//            return (gimbal_ahrs->get_angle() * ahrs_angle_rotation).y;
//        }
//    } else if (motor == SUB_PITCH) {
//        return GimbalIF::feedback[SUB_PITCH]->accumulated_angle() * float(sub_pitch_install) / sub_pitch_deceleration_ratio;
//    }
    return accumulated_angle[motor];
}

float GimbalSKD::get_relative_angle(GimbalBase::motor_id_t motor) {
    if (motor == YAW) {
        return GimbalIF::feedback[YAW]->accumulated_angle() * float(yaw_install) / yaw_deceleration_ratio;
    } else if (motor == PITCH) {
        return GimbalIF::feedback[PITCH]->accumulated_angle() * float(pitch_install) / pitch_deceleration_ratio;
    } else if (motor == SUB_PITCH) {
        return GimbalIF::feedback[SUB_PITCH]->accumulated_angle() * float(sub_pitch_install) / sub_pitch_deceleration_ratio;
    }
    return 0;
}

void GimbalSKD::set_test_status(bool test_status) {
    is_test = test_status;
}

void GimbalSKD::enable_motor(motor_id_t motor) {
    motor_enable[motor] = true;
}

void GimbalSKD::disable_motor(motor_id_t motor) {
    motor_enable[motor] = false;
}
/** @} */