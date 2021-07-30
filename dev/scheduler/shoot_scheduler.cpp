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

ShootSKD::install_direction_t ShootSKD::install_position[3];

ShootSKD::mode_t ShootSKD::mode = FORCED_RELAX_MODE;
bool ShootSKD::motor_enable[3] = {true, true, true};

float ShootSKD::target_angle = 0;
float ShootSKD::actual_angle = 0;
float ShootSKD::fw_target_velocity = 0;
float ShootSKD::target_velocity[3] = {0, 0, 0};
float ShootSKD::actual_velocity[3] = {0, 0, 0};
int ShootSKD::target_current[3] = {0, 0, 0};

PIDController ShootSKD::v2i_pid[3];
PIDController ShootSKD::a2v_pid;

ShootSKD::SKDThread ShootSKD::skd_thread;

void ShootSKD::start(ShootSKD::install_direction_t loader_install_, tprio_t thread_prio) {
    install_position[0] = loader_install_;
    install_position[1] = POSITIVE;
    install_position[2] = NEGATIVE;

    skd_thread.start(thread_prio);
}

void ShootSKD::load_pid_params(pid_params_t loader_a2v_params,
                               pid_params_t loader_v2i_params,
                               pid_params_t fw_left_v2i_params,
                               pid_params_t fw_right_v2i_params) {

    v2i_pid[0].change_parameters(loader_v2i_params);
    a2v_pid.change_parameters(loader_a2v_params);
    v2i_pid[1].change_parameters(fw_left_v2i_params);
    v2i_pid[2].change_parameters(fw_right_v2i_params);
}

void ShootSKD::set_mode(ShootSKD::mode_t skd_mode) {
    mode = skd_mode;
}

ShootSKD::mode_t ShootSKD::get_mode() {
    return mode;
}

void ShootSKD::set_loader_target_angle(float loader_target_angle) {
    target_angle = loader_target_angle;
}

void ShootSKD::set_loader_target_velocity(float degree_per_second) {
    pid_params_t p = a2v_pid.get_parameters();
    p.out_limit = degree_per_second;
    a2v_pid.change_parameters(p);
    a2v_pid.clear_i_out();
}

void ShootSKD::set_friction_wheels(float velocity) {
    fw_target_velocity = velocity;
    target_velocity[1] = velocity * (float) install_position[1];
    target_velocity[2] = velocity * (float) install_position[2];
}

float ShootSKD::get_friction_wheels_target_velocity() {
    return fw_target_velocity;
}

float ShootSKD::get_target_velocity(uint32_t motor_id) {
    return target_velocity[motor_id];
}

int ShootSKD::get_target_current(uint32_t motor_id) {
    return target_current[motor_id];
}

float ShootSKD::get_actual_velocity(uint32_t motor_id) {
    return actual_velocity[motor_id];
}

float ShootSKD::get_loader_target_angle() {
    return target_angle;
}

float ShootSKD::get_loader_accumulated_angle() {
    return actual_angle;
}

void ShootSKD::reset_loader_accumulated_angle() {
    GimbalIF::feedback[GimbalIF::BULLET]->reset_front_angle();
}

void ShootSKD::SKDThread::main() {
    setName("ShootSKD");
    while (!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            if (mode == LIMITED_SHOOTING_MODE) {

                // PID calculation

                // Bullet calculation
                actual_angle = GimbalIF::feedback[GimbalIF::BULLET]->accumulated_angle() * float(install_position[0]);
                target_velocity[0] = a2v_pid.calc(actual_angle, target_angle);

                actual_velocity[0] = GimbalIF::feedback[GimbalIF::BULLET]->actual_velocity * float(install_position[0]);
                target_current[0] = (int) v2i_pid[0].calc(actual_velocity[0], target_velocity[0]);

                // Fraction wheels calculation
                actual_velocity[1] = GimbalIF::feedback[FW_LEFT]->actual_velocity;
                ShootSKD::target_current[1] = (int) v2i_pid[1].calc(actual_velocity[1], target_velocity[1]);

                actual_velocity[2] = GimbalIF::feedback[FW_RIGHT]->actual_velocity;
                ShootSKD::target_current[2] = (int) v2i_pid[2].calc(actual_velocity[2], target_velocity[2]);

            } else if (mode == FORCED_RELAX_MODE) {

                target_current[0] = 0;

                target_current[1] = ABS_IN_RANGE(GimbalIF::feedback[FW_LEFT]->actual_velocity, 100) ?
                                    0 : (int) v2i_pid[1].calc(GimbalIF::feedback[FW_LEFT]->actual_velocity, 0.0f);

                target_current[2] = ABS_IN_RANGE(GimbalIF::feedback[FW_RIGHT]->actual_velocity, 100) ?
                                    0 : (int) v2i_pid[2].calc(GimbalIF::feedback[FW_RIGHT]->actual_velocity, 0.0f);
            }

            for (int i = 0; i <= 2; i++) {
                if (!motor_enable[i]) target_current[i] = 0;
            }

            *GimbalIF::target_current[GimbalIF::BULLET] = target_current[0] * install_position[0];
            *GimbalIF::target_current[GimbalIF::FW_LEFT] = target_current[1];
            *GimbalIF::target_current[GimbalIF::FW_RIGHT] = target_current[2];
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}

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

/** @} */