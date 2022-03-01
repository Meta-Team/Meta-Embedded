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

bool GimbalSKD::motor_enable[GIMBAL_MOTOR_COUNT] = {true};
#ifdef PARAM_ADJUST
bool GimbalSKD::a2v_pid_enabled = true;
#endif
float GimbalSKD::target_angle[GIMBAL_MOTOR_COUNT] = {0};
float GimbalSKD::target_velocity[GIMBAL_MOTOR_COUNT] = {0};
int GimbalSKD::target_current[GIMBAL_MOTOR_COUNT] = {0};
float GimbalSKD::last_angle[GIMBAL_MOTOR_COUNT] = {0};
float GimbalSKD::accumulated_angle[GIMBAL_MOTOR_COUNT] = {0};
GimbalSKD::SKDThread GimbalSKD::skd_thread;
AbstractAHRS *GimbalSKD::gimbal_ahrs = nullptr;
float GimbalSKD::actual_velocity[GIMBAL_MOTOR_COUNT] = {0};

void
GimbalSKD::start(AbstractAHRS *gimbal_ahrs_, const Matrix33 ahrs_angle_rotation_, const Matrix33 ahrs_gyro_rotation_, tprio_t thread_prio) {

    gimbal_ahrs = gimbal_ahrs_;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ahrs_angle_rotation[i][j] = ahrs_angle_rotation_[i][j];
            ahrs_gyro_rotation[i][j] = ahrs_gyro_rotation_[i][j];
        }
    }

    // Initialize last_angle, to use current pointing direction as startup direction
    Vector3D ahrs_angle = ahrs_angle_rotation * gimbal_ahrs->get_angle();

    skd_thread.start(thread_prio);
}

void GimbalSKD::set_mode(GimbalSKD::mode_t skd_mode) {
    mode = skd_mode;
    if(mode == FORCED_RELAX_MODE) {
        CANMotorIF::enable_a2v[CANMotorIF::YAW] = false;
        CANMotorIF::enable_a2v[CANMotorIF::PITCH] = false;
        CANMotorIF::enable_a2v[CANMotorIF::SUB_PITCH] = false;

        CANMotorIF::enable_v2i[CANMotorIF::YAW] = true;
        CANMotorIF::enable_v2i[CANMotorIF::PITCH] = true;
        CANMotorIF::enable_v2i[CANMotorIF::SUB_PITCH] = true;
    } else if (mode == CHASSIS_REF_MODE) {
        CANMotorIF::enable_a2v[CANMotorIF::YAW] = true;
        CANMotorIF::enable_a2v[CANMotorIF::PITCH] = true;
        CANMotorIF::enable_a2v[CANMotorIF::SUB_PITCH] = true;

        CANMotorIF::enable_v2i[CANMotorIF::YAW] = true;
        CANMotorIF::enable_v2i[CANMotorIF::PITCH] = true;
        CANMotorIF::enable_v2i[CANMotorIF::SUB_PITCH] = true;
    } else if (mode == GIMBAL_REF_MODE) {
        CANMotorIF::enable_a2v[CANMotorIF::YAW] = true;
        CANMotorIF::enable_a2v[CANMotorIF::PITCH] = true;
        CANMotorIF::enable_a2v[CANMotorIF::SUB_PITCH] = true;

        CANMotorIF::enable_v2i[CANMotorIF::YAW] = true;
        CANMotorIF::enable_v2i[CANMotorIF::PITCH] = true;
        CANMotorIF::enable_v2i[CANMotorIF::SUB_PITCH] = true;
    }
}

GimbalSKD::mode_t GimbalSKD::get_mode() {
    return mode;
}

void GimbalSKD::set_target_angle(float yaw_target_angle, float pitch_target_angle, float sub_pitch_target_angle) {
    target_angle[GimbalSKD::YAW] = yaw_target_angle;
    target_angle[GimbalSKD::PITCH] = pitch_target_angle;
#if defined(ENABLE_SUBPITCH)
    target_angle[GimbalSKD::SUB_PITCH] = sub_pitch_target_angle;
#endif
}

float GimbalSKD::get_target_angle(GimbalSKD::angle_id_t angleID) {
    return target_angle[angleID];
}

float GimbalSKD::get_AHRS_angle(GimbalSKD::angle_id_t angleID) {
    return accumulated_angle[angleID];
}

float GimbalSKD::get_relatvie_angle(GimbalSKD::angle_id_t angleID) {
    float angle = 0;
    switch (angleID) {
        case YAW:
            angle = CANMotorIF::motor_feedback[CANMotorCFG::YAW].accumulate_angle();
            break;
        case PITCH:
            angle = CANMotorIF::motor_feedback[CANMotorCFG::PITCH].accumulate_angle();
            break;
#if defined(ENABLE_SUBPITCH)
        case SUB_PITCH:
            angle = CANMotorIF::motor_feedback[CANMotorCFG::SUB_PITCH].accumulate_angle();
            break;
#endif
        default:
            return 0;
    }
    return angle;
}

void GimbalSKD::SKDThread::main() {
    setName("GimbalSKD");
    while (!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            /// Update angles
            if (mode == GIMBAL_REF_MODE) {
                // Fetch data
                Vector3D ahrs_angle = ahrs_angle_rotation * gimbal_ahrs->get_angle();
                Vector3D ahrs_gyro = ahrs_gyro_rotation * gimbal_ahrs->get_gyro();

                // Convert AHRS angle and velocity into world frame.
                float angle[2] = {ahrs_angle.x, ahrs_angle.y};
                float velocity[2] = {
                        ahrs_gyro.x * cosf((float)(ahrs_angle.y / 180.0f * M_PI)) +
                        ahrs_gyro.z * sinf((float)(ahrs_angle.y / 180.0f * M_PI)),
                        ahrs_gyro.y};

                // Yaw 0 point calculation
                float yaw_angle_movement = angle[0] - last_angle[YAW];
                last_angle[YAW] = angle[0];
                if (yaw_angle_movement < -200) yaw_angle_movement += 360;
                if (yaw_angle_movement > 200) yaw_angle_movement -= 360;
                accumulated_angle[GimbalSKD::YAW] += yaw_angle_movement;
                actual_velocity[GimbalSKD::YAW] = velocity[0];

                // No need to deal with pitch 0 point
                accumulated_angle[GimbalSKD::PITCH] = angle[1];
                actual_velocity[GimbalSKD::PITCH] = velocity[1];
            } else if (mode == CHASSIS_REF_MODE) {
                accumulated_angle[GimbalSKD::YAW] =
                        CANMotorIF::motor_feedback[CANMotorIF::YAW].accumulate_angle();
                actual_velocity[GimbalSKD::YAW] =
                        CANMotorIF::motor_feedback[CANMotorIF::YAW].actual_velocity;

                accumulated_angle[GimbalSKD::PITCH] =
                        CANMotorIF::motor_feedback[CANMotorIF::YAW].accumulate_angle();
                actual_velocity[GimbalSKD::PITCH] =
                        CANMotorIF::motor_feedback[CANMotorIF::YAW].actual_velocity;
            }
#if defined(ENABLE_SUBPITCH)
            accumulated_angle[SUB_PITCH] =
                    CANMotorIF::motor_feedback[CANMotorIF::SUB_PITCH].accumulate_angle();
            actual_velocity[SUB_PITCH] =
                    CANMotorIF::motor_feedback[CANMotorIF::SUB_PITCH].actual_velocity;
#endif
            /// Set Target
            if (mode == FORCED_RELAX_MODE) {
                CANMotorIF::enable_v2i[CANMotorIF::YAW] = false;
                CANMotorIF::enable_v2i[CANMotorIF::PITCH] = false;
                CANMotorIF::set_current(CANMotorCFG::YAW, 0);
                CANMotorIF::set_current(CANMotorCFG::PITCH, 0);
#if defined(ENABLE_SUBPITCH)
                CANMotorIF::enable_v2i[CANMotorIF::SUB_PITCH] = false;
                CANMotorIF::set_current(CANMotorCFG::SUB_PITCH, 0);
#endif
            } else {
                CANMotorSKD::set_target_angle(CANMotorIF::YAW, target_angle[YAW]);
                CANMotorSKD::set_target_angle(CANMotorIF::PITCH, target_angle[PITCH]);
#if defined(ENABLE_SUBPITCH)
                CANMotorSKD::set_target_angle(CANMotorIF::SUB_PITCH, target_angle[SUB_PITCH]);
#endif
            }
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
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