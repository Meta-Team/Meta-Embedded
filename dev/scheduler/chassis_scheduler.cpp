//
// Created by Administrator on 2019/1/11 0011.
//

/**
 * @file    chassis_scheduler.cpp
 * @brief   Scheduler to control chassis to meet the target, including a thread to invoke PID calculation in period.
 *
 * @addtogroup chassis
 * @{
 */

#include "chassis_scheduler.h"
#include "gimbal_interface.h"
#include "referee_UI_logic.h"

ChassisSKD::mode_t ChassisSKD::mode = FORCED_RELAX_MODE;

float ChassisSKD::target_vx = 0;
float ChassisSKD::target_vy = 0;
float ChassisSKD::target_theta = 0;
float ChassisSKD::actual_theta = 0;

PIDController ChassisSKD::a2v_pid;
PIDController ChassisSKD::v2i_pid[MOTOR_COUNT];

float ChassisSKD::target_w = 0;
float ChassisSKD::target_velocity[MOTOR_COUNT] = {0};
int ChassisSKD::target_current[MOTOR_COUNT] = {0};

float ChassisSKD::wheel_base_ = 0;
float ChassisSKD::wheel_tread_ = 0;
float ChassisSKD::wheel_circumference_ = 0;
float ChassisSKD::w_to_v_ratio_ = 0.0f;
float ChassisSKD::v_to_wheel_angular_velocity_ = 0.0f;
float ChassisSKD::chassis_gimbal_offset_ = 0.0f;

ChassisSKD::install_mode_t ChassisSKD::install_mode_;
GimbalSKD::install_direction_t ChassisSKD::gimbal_yaw_install;

ChassisSKD::SKDThread ChassisSKD::skd_thread;

bool ChassisSKD::motor_enabled = true;

void ChassisSKD::start(float wheel_base, float wheel_tread, float wheel_circumference, install_mode_t install_mode,
                       GimbalSKD::install_direction_t gimbal_yaw_install_, float chassis_gimbal_offset,
                       tprio_t thread_prio) {

    wheel_base_ = wheel_base;
    wheel_tread_ = wheel_tread;
    wheel_circumference_ = wheel_circumference;
    gimbal_yaw_install = gimbal_yaw_install_;

    /*
     * FIXME: in the following lines, it should be 180.0f instead of 360.0f. However, this revision will affect
     *        many aspects, including Theta2W PID parameters for chassis dodge mode. It should be fixed after
     *        season 2019.
     */

    w_to_v_ratio_ = (wheel_base_ + wheel_tread_) / 2.0f / 360.0f * 3.14159f;
    v_to_wheel_angular_velocity_ = (360.0f / wheel_circumference_);
    chassis_gimbal_offset_ = chassis_gimbal_offset;
    install_mode_ = install_mode;

    skd_thread.start(thread_prio);
}

void ChassisSKD::load_pid_params(PIDControllerBase::pid_params_t theta2v_pid_params,
                                 PIDControllerBase::pid_params_t v2i_pid_params) {
    a2v_pid.change_parameters(theta2v_pid_params);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        v2i_pid[i].change_parameters(v2i_pid_params);
    }
}

void ChassisSKD::set_mode(ChassisSKD::mode_t skd_mode) {
    mode = skd_mode;
}

void ChassisSKD::set_target(float vx, float vy, float theta) {
    target_vx = vx;
    target_vy = vy;
    target_theta = theta;
}

void ChassisSKD::set_dodge_target(float vx, float vy, float omega) {
    target_vx = vx;
    target_vy = vy;
    target_w = omega;
}

float ChassisSKD::get_actual_theta() {
    return actual_theta;
}

float ChassisSKD::get_target_theta() {
    return target_theta;
}

void ChassisSKD::velocity_decompose(float vx, float vy, float w) {

    // FR, +vx, -vy, +w
    // FL, +vx, +vy, +w, since the motor is installed in the opposite direction
    // BL, -vx, +vy, +w, since the motor is installed in the opposite direction
    // BR, -vx, -vy, +w

    target_velocity[FR] = install_mode_ * (+vx - vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[FR] = (int) v2i_pid[FR].calc(ChassisIF::feedback[FR]->actual_velocity, target_velocity[FR]);

    target_velocity[FL] = install_mode_ * (+vx + vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[FL] = (int) v2i_pid[FL].calc(ChassisIF::feedback[FL]->actual_velocity, target_velocity[FL]);

    target_velocity[BL] = install_mode_ * (-vx + vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[BL] = (int) v2i_pid[BL].calc(ChassisIF::feedback[BL]->actual_velocity, target_velocity[BL]);

    target_velocity[BR] = install_mode_ * (-vx - vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[BR] = (int) v2i_pid[BR].calc(ChassisIF::feedback[BR]->actual_velocity, target_velocity[BR]);
}

float ChassisSKD::get_actual_velocity(ChassisBase::motor_id_t motor_id) {
    return ChassisIF::feedback[motor_id]->actual_velocity;
}

void ChassisSKD::SKDThread::main() {
    setName("ChassisSKD");
    while (!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            if ((mode == GIMBAL_COORDINATE_MODE) || (mode == ANGULAR_VELOCITY_DODGE_MODE)) {

                actual_theta = GimbalIF::feedback[GimbalIF::YAW]->actual_angle * (float) gimbal_yaw_install;

                if (mode == GIMBAL_COORDINATE_MODE) {
                    if (ABS(actual_theta - target_theta) < THETA_DEAD_ZONE) {
                        target_w = 0;
                    } else {
                        target_w = a2v_pid.calc(actual_theta, target_theta);
                    }
                }

                velocity_decompose(
                        target_vx * cosf(actual_theta / 180.0f * PI) - target_vy * sinf(actual_theta / 180.0f * PI)
                        - target_w / 180.0f * PI * chassis_gimbal_offset_,

                        target_vx * sinf(actual_theta / 180.0f * PI) + target_vy * cosf(actual_theta / 180.0f * PI),

                        target_w);

            } else if (mode == FORCED_RELAX_MODE) {

                for (size_t i = 0; i < MOTOR_COUNT; i++) {
                    target_current[i] = 0;
                }

            }

            if (!motor_enabled) for (int &c : target_current) c = 0;


            // Send currents
            for (size_t i = 0; i < MOTOR_COUNT; i++) {
                *ChassisIF::target_current[i] = target_current[i];
            }
            ChassisIF::clip_chassis_current();
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}

const Shell::Command ChassisSKD::shellCommands[] = {
        {"_c",              nullptr,                                        ChassisSKD::cmdInfo,           nullptr},
        {"_c_enable_fb",    "Channel/All Feedback{Disabled,Enabled}",       ChassisSKD::cmdEnableFeedback, nullptr},
        {"_c_pid",          "Channel [kp] [ki] [kd] [i_limit] [out_limit]", ChassisSKD::cmdPID,            nullptr},
        {"_c_enable_motor", "All Motor{Disabled,Enabled}",                  ChassisSKD::cmdEnableMotor,    nullptr},
        {nullptr,           nullptr,                                        nullptr,                       nullptr}
};

DEF_SHELL_CMD_START(ChassisSKD::cmdInfo)
    Shell::printf("_c:Chassis" ENDL);
    Shell::printf("_c/Front_Right:Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_c/Front_Left:Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_c/Back_Left:Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_c/Back_Right:Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_c/Theta:Angle{Actual,Target}" ENDL);
    return true;
DEF_SHELL_CMD_END

static bool feedbackEnabled[ChassisSKD::MOTOR_COUNT + 1] = {false, false, false, false, false};

void ChassisSKD::cmdFeedback(void *) {
    for (int i = 0; i <= MOTOR_COUNT; i++) {
        if (feedbackEnabled[i]) {
            Shell::printf("_c%d %.2f %.2f %d %d" ENDL, i,
                          ChassisIF::feedback[i]->actual_velocity, target_velocity[i],
                          ChassisIF::feedback[i]->actual_current, *ChassisIF::target_current[i]);
        }
    }
    if (feedbackEnabled[4]) {
        Shell::printf("_c4 %.2f %.2f" ENDL, actual_theta, target_theta);
    }
}

DEF_SHELL_CMD_START(ChassisSKD::cmdEnableFeedback)
    int id;
    bool enabled;
    if (!Shell::parseIDAndBool(argc, argv, 5, id, enabled)) return false;
    if (id == -1) {
        for (bool &e : feedbackEnabled) e = enabled;
    } else {
        feedbackEnabled[id] = enabled;
    }
    return true;
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(ChassisSKD::cmdEnableMotor)
    int id;
    bool enabled;
    if (!Shell::parseIDAndBool(argc, argv, 0, id, enabled)) return false;
    if (id == -1) {
        motor_enabled = enabled;
        return true;
    } else {
        return false;
    }
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(ChassisSKD::cmdPID)
    if (argc < 1) return false;

    unsigned id = Shell::atoi(argv[0]);
    if (id >= MOTOR_COUNT + 1) return false;

    PIDController *pid = id == MOTOR_COUNT + 1 ? &a2v_pid : &v2i_pid[id];
    if (argc == 1) {
        pid_params_t params = pid->get_parameters();
        Shell::printf("_c_pid %u %.2f %.2f %.2f %.2f %.2f" SHELL_NEWLINE_STR,
                      id, params.kp, params.ki, params.kd, params.i_limit, params.out_limit);
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

/** @} */