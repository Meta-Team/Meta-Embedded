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

#include "mecanum_chassis_scheduler.h"

float MecanumChassisSKD::target_vx = 0.0f;
float MecanumChassisSKD::target_vy = 0.0f;
float MecanumChassisSKD::target_omega = 0.0f;

MecanumChassisSKD::install_mode_t MecanumChassisSKD::install_mode = POSITIVE;
float MecanumChassisSKD::w_to_v_ratio = 0.0f;
float MecanumChassisSKD::v_to_wheel_angular_velocity = 0.0f;

MecanumChassisSKD::SKDThread MecanumChassisSKD::skd_thread;

void MecanumChassisSKD::init(tprio_t skd_prio, float wheel_base, float wheel_thread, float wheel_circumference) {
    skd_thread.start(skd_prio);
    w_to_v_ratio = (wheel_base + wheel_thread) / 2.0f / 360.0f * 3.14159f;
    v_to_wheel_angular_velocity = (360.0f / wheel_circumference);
}

void MecanumChassisSKD::set_target(float vx, float vy, float omega) {
    target_vx = vx;
    target_vy = vy;
    target_omega = omega;
    // For DODGE_MODE keep current target_theta unchanged
}

void MecanumChassisSKD::SKDThread::main() {
    setName("ChassisSKDThread");
    while(!shouldTerminate()) {
        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            CANMotorSKD::set_target_vel(CANMotorCFG::FR, (float)install_mode *
                                                         (target_vx-target_vy + target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
            CANMotorSKD::set_target_vel(CANMotorCFG::FL, (float)install_mode *
                                                         (target_vx+target_vy + target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
            CANMotorSKD::set_target_vel(CANMotorCFG::BL, (float)install_mode *
                                                         (-target_vx+target_vy+ target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
            CANMotorSKD::set_target_vel(CANMotorCFG::BR, (float)install_mode *
                                                         (-target_vx-target_vy+ target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---
        sleep(TIME_MS2I(SKD_INTERVAL));
    }

}
/// TODO: Re-Enable These functions.
#ifdef ENABLE_CHASSIS_SHELL
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
#endif

/** @} */