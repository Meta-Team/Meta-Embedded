//
// Created by liuzikai on 2019-05-01.
// Mo Kanya revised the friction wheel part.
//

/**
 * @file    shoot_scheduler.cpp
 * @brief   Scheduler to control shooter to meet the target, including a thread to invoke PID calculation in period.
 *
 * @addtogroup shoot
 * @{
 */

#include "shoot_scheduler.h"

ShootSKD::mode_t ShootSKD::mode = FORCED_RELAX_MODE;

float ShootSKD::bullet_target_angle = 0.0f;
float ShootSKD::fw_target_velocity = 0;

ShootSKD::SKDThread ShootSKD::skd_thread;

void ShootSKD::start(tprio_t thread_prio) {
    skd_thread.start(thread_prio);
}

void ShootSKD::set_mode(ShootSKD::mode_t skd_mode) {
    mode = skd_mode;
    if(skd_mode == FORCED_RELAX_MODE) {
        CANMotorIF::enable_a2v[CANMotorCFG::BULLET_LOADER] = false;
        CANMotorIF::enable_v2i[CANMotorCFG::BULLET_LOADER] = false;
        CANMotorIF::enable_v2i[CANMotorCFG::FW_UP] = true;
        CANMotorIF::enable_v2i[CANMotorCFG::FW_DOWN] = true;
    } else {
        CANMotorIF::enable_a2v[CANMotorCFG::BULLET_LOADER] = true;
        CANMotorIF::enable_v2i[CANMotorCFG::BULLET_LOADER] = true;
        CANMotorIF::enable_v2i[CANMotorCFG::FW_UP] = true;
        CANMotorIF::enable_v2i[CANMotorCFG::FW_DOWN] = true;
    }
}

ShootSKD::mode_t ShootSKD::get_mode() {
    return mode;
}

void ShootSKD::set_loader_target_angle(float loader_target_angle) {
    bullet_target_angle = loader_target_angle;
}

void ShootSKD::set_friction_wheels(float velocity) {
    fw_target_velocity = velocity;
}

float ShootSKD::get_friction_wheels_target_velocity() {
    return fw_target_velocity;
}

float ShootSKD::get_loader_target_angle() {
    return bullet_target_angle;
}

float ShootSKD::get_loader_accumulated_angle() {
    return CANMotorIF::motor_feedback[CANMotorCFG::BULLET_LOADER].accumulate_angle();
}

void ShootSKD::reset_loader_accumulated_angle() {
    CANMotorIF::motor_feedback[CANMotorCFG::BULLET_LOADER].reset_accumulate_angle();
}

void ShootSKD::set_loader_target_velocity(float degree_per_second) {
    CANMotorIF::a2vParams[CANMotorCFG::BULLET_LOADER].out_limit = degree_per_second;
}

void ShootSKD::SKDThread::main() {
    setName("ShootSKD");
    while (!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {

            if (mode == LIMITED_SHOOTING_MODE) {

                CANMotorSKD::set_target_angle(CANMotorCFG::BULLET_LOADER, bullet_target_angle);
                CANMotorSKD::set_target_vel(CANMotorCFG::FW_UP, -fw_target_velocity);
                CANMotorSKD::set_target_vel(CANMotorCFG::FW_DOWN, fw_target_velocity);

            } else if (mode == FORCED_RELAX_MODE) {

                CANMotorSKD::set_target_current(CANMotorCFG::BULLET_LOADER, 0);

                if(ABS_IN_RANGE(CANMotorIF::motor_feedback[CANMotorCFG::FW_UP].actual_velocity, 500)) {
                    CANMotorIF::enable_v2i[CANMotorCFG::FW_UP] = false;
                    CANMotorSKD::set_target_current(CANMotorCFG::FW_UP, 0);
                } else {
                    CANMotorSKD::set_target_vel(CANMotorCFG::FW_UP, 0);
                }
                if(ABS_IN_RANGE(CANMotorIF::motor_feedback[CANMotorCFG::FW_DOWN].actual_velocity, 500)) {
                    CANMotorIF::enable_v2i[CANMotorCFG::FW_DOWN] = false;
                    CANMotorSKD::set_target_current(CANMotorCFG::FW_DOWN, 0);
                } else {
                    CANMotorSKD::set_target_vel(CANMotorCFG::FW_DOWN, 0);
                }
            }
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}
/// TODO: Re-enable shell commands
/***
const Shell::Command ShootSKD::shellCommands[] = {
        {"_s",              nullptr,                                             ShootSKD::cmdInfo,           nullptr},
        {"_s_enable_fb",    "Channel/All Feedback{Disabled,Enabled}",            ShootSKD::cmdEnableFeedback, nullptr},
        {"_s_pid",          "Channel PID{A2V,V2I} [kp] [ki] [kd] [i_limit] [out_limit]", ShootSKD::cmdPID,            nullptr},
        {"_s_enable_motor", "Channel/All Motor{Disabled,Enabled}",               ShootSKD::cmdEnableMotor,    nullptr},
        {nullptr,           nullptr,                                             nullptr,                     nullptr}
};

DEF_SHELL_CMD_START(ShootSKD::cmdInfo)
    Shell::printf("_s:Shoot" ENDL);
    Shell::printf("_s/Bullet:Angle{Actual,Target} Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_s/FW_Left:Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_s/FW_Right:Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    return true;
DEF_SHELL_CMD_END


static bool feedbackEnabled[3] = {false, false, false};

void ShootSKD::cmdFeedback(void *) {
    if (feedbackEnabled[0]) {
        Shell::printf("_s0 %.2f %.2f %.2f %.2f %d %d" ENDL,
                      actual_angle, target_angle,
                      actual_velocity[0], target_velocity[0],
                      GimbalIF::feedback[3]->actual_current, *GimbalIF::target_current[3]);
    }
    for (int i = 1; i < 3; i++) {
        if (feedbackEnabled[i]) {
            Shell::printf("_s%d %.2f %.2f %d %d" ENDL, i,
                          actual_velocity[i], target_velocity[i],
                          GimbalIF::feedback[3 + i]->actual_current, *GimbalIF::target_current[3 + i]);
        }
    }
}

DEF_SHELL_CMD_START(ShootSKD::cmdEnableFeedback)
    int id;
    bool enabled;
    if (!Shell::parseIDAndBool(argc, argv, 3, id, enabled)) return false;
    if (id == -1) {
        for (bool &e : feedbackEnabled) e = enabled;
    } else {
        feedbackEnabled[id] = enabled;
    }
    return true;
DEF_SHELL_CMD_END


DEF_SHELL_CMD_START(ShootSKD::cmdEnableMotor)
    int id;
    bool enabled;
    if (!Shell::parseIDAndBool(argc, argv, 3, id, enabled)) return false;
    if (id == -1) {
        for (bool &e : motor_enable) e = enabled;
    } else {
        motor_enable[id] = enabled;
    }
    return true;
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(ShootSKD::cmdPID)
    if (argc < 2) return false;

    unsigned id = Shell::atoi(argv[0]);
    if (id >= 3) return false;

    unsigned pidType = Shell::atoi(argv[1]);
    if (pidType > 1) return false;
    if (id != 0 && pidType == 0) return false;

    PIDController *pid = pidType == 0 ? &a2v_pid : &v2i_pid[id];
    if (argc == 2) {
        pid_params_t params = pid->get_parameters();
        Shell::printf("_s_pid %u %u %.2f %.2f %.2f %.2f %.2f" SHELL_NEWLINE_STR,
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

***/

/** @} */