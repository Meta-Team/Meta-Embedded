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
GimbalSKD::angle_mode_t GimbalSKD::angle_mode = ABS_ANGLE_MODE;
GimbalSKD::install_direction_t GimbalSKD::yaw_install;
GimbalSKD::install_direction_t GimbalSKD::pitch_install;
GimbalSKD::install_direction_t GimbalSKD::sub_pitch_install;

bool GimbalSKD::motor_enable[3] = {true, true, true};
#ifdef PARAM_ADJUST
bool GimbalSKD::a2v_pid_enabled = true;
#endif
float GimbalSKD::target_angle[3] = {0, 0, 0};
float GimbalSKD::target_velocity[3] = {0, 0, 0};
int GimbalSKD::target_current[3] = {0, 0, 0};
float GimbalSKD::last_angle[3] = {0, 0, 0};
float GimbalSKD::accumulated_angle[3] = {0, 0, 0};
PIDController GimbalSKD::a2v_pid[3];
PIDController GimbalSKD::v2i_pid[3];
GimbalSKD::SKDThread GimbalSKD::skd_thread;
AbstractAHRS *GimbalSKD::gimbal_ahrs = nullptr;
float GimbalSKD::actual_velocity[3] = {0, 0, 0};

void
GimbalSKD::start(AbstractAHRS *gimbal_ahrs_, const Matrix33 ahrs_angle_rotation_, const Matrix33 ahrs_gyro_rotation_,
                 install_direction_t yaw_install_, install_direction_t pitch_install_,
                 install_direction_t sub_pitch_install_, tprio_t thread_prio, angle_mode_t angle_mode_) {

    gimbal_ahrs = gimbal_ahrs_;
    yaw_install = yaw_install_;
    pitch_install = pitch_install_;
    sub_pitch_install = sub_pitch_install_;
    angle_mode = angle_mode_;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ahrs_angle_rotation[i][j] = ahrs_angle_rotation_[i][j];
            ahrs_gyro_rotation[i][j] = ahrs_gyro_rotation_[i][j];
        }
    }

    // Initialize last_angle, to use current pointing direction as startup direction
    Vector3D ahrs_angle = ahrs_angle_rotation * gimbal_ahrs->get_angle();

    if (angle_mode == ABS_ANGLE_MODE) {
        last_angle[YAW] = ahrs_angle.x;
        last_angle[PITCH] = ahrs_angle.y;
    } else {
        last_angle[YAW] = GimbalIF::feedback[YAW]->accumulated_angle() * yaw_install;
        last_angle[PITCH] = GimbalIF::feedback[PITCH]->accumulated_angle() * pitch_install;
    }
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

            // Update angles
            if (angle_mode == ABS_ANGLE_MODE) {

                // Fetch data from ahrs
                Vector3D ahrs_angle = ahrs_angle_rotation * gimbal_ahrs->get_angle();
                Vector3D ahrs_gyro = ahrs_gyro_rotation * gimbal_ahrs->get_gyro();

                // TODO: document calculations here
                float angle[2] = {ahrs_angle.x, ahrs_angle.y};
                float velocity[2] = {
                        ahrs_gyro.x * cosf(ahrs_angle.y / 180.0f * M_PI) +
                        ahrs_gyro.z * sinf(ahrs_angle.y / 180.0f * M_PI),
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
                accumulated_angle[YAW] += yaw_angle_movement;
                actual_velocity[YAW] = velocity[0];

                last_angle[PITCH] = angle[1];
                accumulated_angle[PITCH] = angle[1];
                actual_velocity[PITCH] = velocity[1];

            } else if (angle_mode == RELATIVE_ANGLE_MODE) {

                // Fetch data from GimbalIF
                last_angle[YAW] = GimbalIF::feedback[YAW]->accumulated_angle();
                accumulated_angle[YAW] = GimbalIF::feedback[YAW]->accumulated_angle();
                actual_velocity[YAW] = GimbalIF::feedback[YAW]->actual_velocity;

                last_angle[PITCH] = GimbalIF::feedback[PITCH]->accumulated_angle();
                accumulated_angle[PITCH] = GimbalIF::feedback[PITCH]->accumulated_angle();
                actual_velocity[PITCH] = GimbalIF::feedback[PITCH]->actual_velocity;
            }

            last_angle[SUB_PITCH] = GimbalIF::feedback[SUB_PITCH]->accumulated_angle();
            accumulated_angle[SUB_PITCH] = GimbalIF::feedback[SUB_PITCH]->accumulated_angle();
            actual_velocity[SUB_PITCH] = GimbalIF::feedback[SUB_PITCH]->actual_velocity;

            // Update target current
            if (mode == FORCED_RELAX_MODE) {

                target_current[YAW] = target_current[PITCH] = target_current[SUB_PITCH] = 0;

            } else {

#ifdef PARAM_ADJUST
                if (a2v_pid_enabled) {
#endif
                    target_velocity[YAW] = a2v_pid[YAW].calc(accumulated_angle[YAW], target_angle[YAW]);
                    target_velocity[PITCH] = a2v_pid[PITCH].calc(accumulated_angle[PITCH], target_angle[PITCH]);
                    target_velocity[SUB_PITCH] = a2v_pid[SUB_PITCH].calc(accumulated_angle[SUB_PITCH],
                                                                         target_angle[SUB_PITCH]);
#ifdef PARAM_ADJUST
                }
#endif

                target_current[YAW] = (int) v2i_pid[YAW].calc(actual_velocity[YAW], target_velocity[YAW]);
                target_current[PITCH] = (int) v2i_pid[PITCH].calc(actual_velocity[PITCH], target_velocity[PITCH]);
                target_current[SUB_PITCH] = (int) v2i_pid[SUB_PITCH].calc(actual_velocity[SUB_PITCH],
                                                                          target_velocity[SUB_PITCH]);
            }

            for (int i = YAW; i <= SUB_PITCH; i++) {
                if (!motor_enable[i]) target_current[i] = 0;
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

float GimbalSKD::get_relative_angle(GimbalBase::motor_id_t motor) {
    if (motor == YAW) {
        return GimbalIF::feedback[YAW]->accumulated_angle() * float(yaw_install);
    } else if (motor == PITCH) {
        return GimbalIF::feedback[PITCH]->accumulated_angle() * float(pitch_install);
    } else if (motor == SUB_PITCH) {
        return GimbalIF::feedback[SUB_PITCH]->accumulated_angle() * float(sub_pitch_install);
    }
    return 0;
}

const Shell::Command GimbalSKD::shellCommands[] = {
        {"_g",              nullptr,                                                     GimbalSKD::cmdInfo,           nullptr},
        {"_g_enable_fb",    "Channel/All Feedback{Disabled,Enabled}",                    GimbalSKD::cmdEnableFeedback, nullptr},
        {"_g_pid",          "Channel PID{A2V,V2I} [kp] [ki] [kd] [i_limit] [out_limit]", GimbalSKD::cmdPID,            nullptr},
        {"_g_enable_motor", "Channel/All Motor{Disabled,Enabled}",                       GimbalSKD::cmdEnableMotor,    nullptr},
        {"_g_echo_raw",     "Channel",                                                   GimbalSKD::cmdEchoRaw,        nullptr},
        {nullptr,           nullptr,                                                     nullptr,                      nullptr}
};

DEF_SHELL_CMD_START(GimbalSKD::cmdInfo)
    Shell::printf("_g:Gimbal" ENDL);
    Shell::printf("_g/Yaw:Angle{Actual,Target} Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_g/Pitch:Angle{Actual,Target} Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_g/Sub_Pitch:Angle{Actual,Target} Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    return true;
DEF_SHELL_CMD_END

static bool feedbackEnabled[3] = {false, false, false};

void GimbalSKD::cmdFeedback(void *) {
    for (int i = YAW; i <= SUB_PITCH; i++) {
        if (feedbackEnabled[i]) {
            Shell::printf("_g%d %.2f %.2f %.2f %.2f %d %d" ENDL, i,
                          accumulated_angle[i], target_angle[i],
                          actual_velocity[i], target_velocity[i],
                          GimbalIF::feedback[i]->actual_current, *GimbalIF::target_current[i]);
        }
    }
}

DEF_SHELL_CMD_START(GimbalSKD::cmdEnableFeedback)
    int id;
    bool enabled;
    if (!Shell::parseIDAndBool(argc, argv, 3, id, enabled)) return false;
    if (id == -1) {
        for (int i = YAW; i <= SUB_PITCH; i++) feedbackEnabled[i] = enabled;
    } else {
        feedbackEnabled[id] = enabled;
    }
    return true;
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(GimbalSKD::cmdEnableMotor)
    int id;
    bool enabled;
    if (!Shell::parseIDAndBool(argc, argv, 3, id, enabled)) return false;
    if (id == -1) {
        for (int i = YAW; i <= SUB_PITCH; i++) motor_enable[i] = enabled;
    } else {
        motor_enable[id] = enabled;
    }
    return true;
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(GimbalSKD::cmdPID)
    if (argc < 2) return false;

    unsigned id = Shell::atoi(argv[0]);
    if (id > SUB_PITCH) return false;

    unsigned pidType = Shell::atoi(argv[1]);
    if (pidType > 1) return false;

    PIDController *pid = pidType == 0 ? &a2v_pid[id] : &v2i_pid[id];
    if (argc == 2) {
        pid_params_t params = pid->get_parameters();
        Shell::printf("_g_pid %u %u %.2f %.2f %.2f %.2f %.2f" SHELL_NEWLINE_STR,
                      id, pidType, params.kp, params.ki, params.kd, params.i_limit, params.out_limit);
    } else if (argc == 7) {
        pid->change_parameters({Shell::atof(argv[2]),
                                Shell::atof(argv[3]),
                                Shell::atof(argv[4]),
                                Shell::atof(argv[5]),
                                Shell::atof(argv[6])});
    } else {
        return false;
    }

    return true;
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(GimbalSKD::cmdEchoRaw)
    if (argc != 1) return false;
    unsigned id = Shell::atoi(argv[0]);
    if (id >= MOTOR_COUNT) return false;
    Shell::printf("%d" ENDL, GimbalIF::feedback[id]->last_angle_raw);
    return true;
DEF_SHELL_CMD_END

/** @} */