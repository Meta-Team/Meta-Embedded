//
// Created by liuzikai on 2019-01-05.
//

/**
 * @file    gimbal_scheduler.cpp
 * @brief   Scheduler to control gimbal to meet the target, including a thread to invoke PID calculation in period.
 * @addtogroup gimbal
 * @{
 */

#include "gimbal_scheduler.h"
#include <cmath>

Matrix33 GimbalSKD::ahrs_angle_rotation;
Matrix33 GimbalSKD::ahrs_gyro_rotation;
GimbalSKD::mode_t GimbalSKD::mode = FORCED_RELAX_MODE;

float GimbalSKD::target_angle[GIMBAL_MOTOR_COUNT] = {0};
float GimbalSKD::last_angle[GIMBAL_MOTOR_COUNT] = {0};
float GimbalSKD::feedback_angle[GIMBAL_MOTOR_COUNT] = {0};
GimbalSKD::SKDThread GimbalSKD::skd_thread;
AbstractAHRS *GimbalSKD::gimbal_ahrs = nullptr;
float GimbalSKD::feedback_velocity[GIMBAL_MOTOR_COUNT] = {0};
#if ENABLE_SUBPITCH == FALSE
GimbalSKD::install_direction_t GimbalSKD::install_direction[GIMBAL_MOTOR_COUNT] = {POSITIVE, POSITIVE};
#else
GimbalSKD::install_direction_t GimbalSKD::install_direction[GIMBAL_MOTOR_COUNT] = {POSITIVE, POSITIVE, POSITIVE};
#endif

void GimbalSKD::start(AbstractAHRS *gimbal_ahrs_, const Matrix33 ahrs_angle_rotation_, const Matrix33 ahrs_gyro_rotation_, tprio_t thread_prio) {

    gimbal_ahrs = gimbal_ahrs_;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ahrs_angle_rotation[i][j] = ahrs_angle_rotation_[i][j];
            ahrs_gyro_rotation[i][j] = ahrs_gyro_rotation_[i][j];
        }
    }

    // Initialize last_angle, to use current pointing direction as startup direction
    Vector3D ahrs_angle = ahrs_angle_rotation * gimbal_ahrs->get_angle();
    last_angle[YAW] = ahrs_angle.x * (float)install_direction[YAW];
    last_angle[PITCH] = ahrs_angle.y * (float)install_direction[PITCH];
    skd_thread.start(thread_prio);
}

void GimbalSKD::set_target_angle(float yaw_target_angle, float pitch_target_angle, float sub_pitch_target_angle) {
    target_angle[GimbalSKD::YAW] = yaw_target_angle;
    target_angle[GimbalSKD::PITCH] = pitch_target_angle;
#if ENABLE_SUBPITCH == TRUE
    target_angle[GimbalSKD::SUB_PITCH] = sub_pitch_target_angle;
#endif
}

float GimbalSKD::get_target_angle(GimbalSKD::angle_id_t angle) {
    return target_angle[angle];
}

float GimbalSKD::get_feedback_angle(GimbalSKD::angle_id_t angle) {
    return feedback_angle[angle];
}

void GimbalSKD::SKDThread::main() {
    setName("GimbalSKD");
    while (!shouldTerminate()) {

        chSysLock();  /// S-Lock State
        {
            /// Update angles
            if (mode == GIMBAL_REF_MODE) {
                /// Use AHRS angles for gimbal feedback
                // In S-Lock State, ahrs feedback will not change during performing calculation.
                Vector3D ahrs_angle = ahrs_angle_rotation * gimbal_ahrs->get_angle() * (float)install_direction[YAW];
                Vector3D ahrs_gyro  = ahrs_gyro_rotation  * gimbal_ahrs->get_gyro()  * (float)install_direction[YAW];
                // Yaw 0 point calculation.
                float yaw_angle_movement            =   ahrs_angle.x - last_angle[YAW];
                last_angle[YAW]                     =   ahrs_angle.x;
                if (yaw_angle_movement < -200)          yaw_angle_movement   += 360;
                if (yaw_angle_movement >  200)          yaw_angle_movement   -= 360;
                // Convert AHRS angle and velocity into world frame.
                feedback_angle[GimbalSKD::YAW]      +=  yaw_angle_movement;
                feedback_velocity[GimbalSKD::YAW]   =   ahrs_gyro.x * cosf((float)(ahrs_angle.y / 180.0f * M_PI)) +
                                                        ahrs_gyro.z * sinf((float)(ahrs_angle.y / 180.0f * M_PI));
                feedback_angle[GimbalSKD::PITCH]    =   ahrs_angle.y;
                feedback_velocity[GimbalSKD::PITCH] =   ahrs_gyro.y;
            } else if (mode == CHASSIS_REF_MODE) {
                /// Use motor encoder angles for gimbal feedback
                feedback_angle[GimbalSKD::YAW] =
                        CANMotorIF::motor_feedback[CANMotorCFG::YAW].accumulate_angle();
                feedback_velocity[GimbalSKD::YAW] =
                        CANMotorIF::motor_feedback[CANMotorCFG::YAW].actual_velocity;
                feedback_angle[GimbalSKD::PITCH] =
                        CANMotorIF::motor_feedback[CANMotorCFG::YAW].accumulate_angle();
                feedback_velocity[GimbalSKD::PITCH] =
                        CANMotorIF::motor_feedback[CANMotorCFG::YAW].actual_velocity;
            }
#if ENABLE_SUBPITCH == TRUE
            feedback_angle[SUB_PITCH] =
                    CANMotorIF::motor_feedback[CANMotorCFG::SUB_PITCH].accumulate_angle();
            feedback_velocity[SUB_PITCH] =
                    CANMotorIF::motor_feedback[CANMotorCFG::SUB_PITCH].actual_velocity;
#endif
            /// Set Target
            if (mode == FORCED_RELAX_MODE) {
                /// Safe mode
                CANMotorCFG::enable_v2i[CANMotorCFG::YAW] = false;
                CANMotorCFG::enable_v2i[CANMotorCFG::PITCH] = false;
                CANMotorSKD::set_target_current(CANMotorCFG::YAW, 0);
                CANMotorSKD::set_target_current(CANMotorCFG::PITCH, 0);
#if ENABLE_SUBPITCH == TRUE
                CANMotorCFG::enable_v2i[CANMotorCFG::SUB_PITCH] = false;
                CANMotorSKD::set_target_current(CANMotorCFG::SUB_PITCH, 0);
#endif
            } else {
                /// Let CANMotorSKD perform the PID calculation.
                CANMotorSKD::set_target_angle(CANMotorCFG::YAW, target_angle[YAW]);
                CANMotorSKD::set_target_angle(CANMotorCFG::PITCH, target_angle[PITCH]);
#if ENABLE_SUBPITCH == TRUE
                CANMotorSKD::set_target_angle(CANMotorCFG::SUB_PITCH, target_angle[SUB_PITCH]);
#endif
            }
        }
        chSysUnlock();  /// EXIT S-Locked state

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}

void GimbalSKD::set_mode(GimbalSKD::mode_t skd_mode) {
    mode = skd_mode;
    switch (skd_mode) {

        case FORCED_RELAX_MODE:
            CANMotorCFG::enable_a2v[CANMotorCFG::YAW] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::YAW] = false;
            CANMotorSKD::set_target_current(CANMotorCFG::YAW, 0);

            CANMotorCFG::enable_a2v[CANMotorCFG::PITCH] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::PITCH] = false;
            CANMotorSKD::set_target_current(CANMotorCFG::PITCH, 0);
#if ENABLE_SUBPITCH == TRUE
            CANMotorCFG::enable_a2v[CANMotorCFG::SUB_PITCH] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::SUB_PITCH] = false;
            CANMotorSKD::set_target_current(CANMotorCFG::SUB_PITCH, 0);
#endif
            break;

        case GIMBAL_REF_MODE:
            CANMotorSKD::register_custom_feedback(&feedback_angle[GimbalSKD::YAW],
                                                  CANMotorSKD::angle,
                                                  CANMotorCFG::YAW);
            CANMotorSKD::register_custom_feedback(&feedback_velocity[GimbalSKD::YAW],
                                                  CANMotorSKD::velocity,
                                                  CANMotorCFG::YAW);
            CANMotorSKD::register_custom_feedback(&feedback_angle[GimbalSKD::PITCH],
                                                  CANMotorSKD::angle,
                                                  CANMotorCFG::PITCH);
            CANMotorSKD::register_custom_feedback(&feedback_velocity[GimbalSKD::PITCH],
                                                  CANMotorSKD::velocity,
                                                  CANMotorCFG::PITCH);
            CANMotorCFG::enable_a2v[CANMotorCFG::YAW] = true;
            CANMotorCFG::enable_v2i[CANMotorCFG::YAW] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::PITCH] = true;
            CANMotorCFG::enable_v2i[CANMotorCFG::PITCH] = true;
            break;
        case CHASSIS_REF_MODE:
            CANMotorSKD::unregister_custom_feedback(CANMotorSKD::angle,
                                                    CANMotorCFG::YAW);
            CANMotorSKD::unregister_custom_feedback(CANMotorSKD::velocity,
                                                    CANMotorCFG::YAW);
            CANMotorSKD::unregister_custom_feedback(CANMotorSKD::angle,
                                                    CANMotorCFG::PITCH);
            CANMotorSKD::unregister_custom_feedback(CANMotorSKD::velocity,
                                                    CANMotorCFG::PITCH);

            CANMotorCFG::enable_a2v[CANMotorCFG::YAW] = true;
            CANMotorCFG::enable_v2i[CANMotorCFG::YAW] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::PITCH] = true;
            CANMotorCFG::enable_v2i[CANMotorCFG::PITCH] = true;
#if ENABLE_SUBPITCH == TRUE
            /// TODO: enable sub pitch motor, so the variables should be both true.
            CANMotorCFG::enable_a2v[CANMotorCFG::SUB_PITCH] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::SUB_PITCH] = false;
#endif
            break;
    }
}

GimbalSKD::mode_t GimbalSKD::get_mode() {
    return mode;
}

void GimbalSKD::set_installation(GimbalSKD::angle_id_t angle, GimbalSKD::install_direction_t install_direction_) {
    GimbalSKD::install_direction[angle] = install_direction_;
}

float GimbalSKD::get_feedback_velocity(GimbalSKD::angle_id_t angle) {
    return feedback_velocity[angle];
}

/// TODO: re-enable shell functions

/***

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
                          Motor::feedback[i]->actual_current, *GimbalIF::target_current[i]);
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
***/
/** @} */