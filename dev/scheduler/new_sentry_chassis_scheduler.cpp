//
// Created by kerui on 2021/7/29.
//

#include "new_sentry_chassis_scheduler.h"

SChassisSKD::mode_t SChassisSKD::mode;
float SChassisSKD::target_location_ = 0;
float SChassisSKD::accumulated_displacement[MOTOR_COUNT] = {0, 0};
float SChassisSKD::actual_velocity[MOTOR_COUNT] = {0,0};
float SChassisSKD::target_velocity[MOTOR_COUNT] = {0,0};
int SChassisSKD::target_current[MOTOR_COUNT] = {0};
PIDController SChassisSKD::a2v_pid[MOTOR_COUNT];
PIDController SChassisSKD::v2i_pid[MOTOR_COUNT];
SChassisSKD::SKDThread SChassisSKD::skd_thread;
bool SChassisSKD::motor_enabled = true;

void SChassisSKD::start(tprio_t thread_prio) {
    skd_thread.start(thread_prio);
}

void SChassisSKD::load_pid_params(pid_params_t a2v_pid_params, pid_params_t v2i_pid_params) {
    a2v_pid[R].change_parameters(a2v_pid_params);
    a2v_pid[L].change_parameters(a2v_pid_params);
    v2i_pid[R].change_parameters(v2i_pid_params);
    v2i_pid[L].change_parameters(v2i_pid_params);
}

void SChassisSKD::set_mode(mode_t skd_mode) {
    mode = skd_mode;
    SChassisIF::feedback[R]->reset_front_angle();
    SChassisIF::feedback[L]->reset_front_angle();
}

void SChassisSKD::set_target(float target_location) {
    target_location_ = target_location;
}

void SChassisSKD::SKDThread::main() {
    setName("SChassisSKD");
    while (!shouldTerminate()) {
        accumulated_displacement[R] = SChassisIF::feedback[R]->accumulated_angle() / 360.f * DISPLACEMENT_PER_ROUND;
        accumulated_displacement[L] = SChassisIF::feedback[L]->accumulated_angle() / 360.f * DISPLACEMENT_PER_ROUND;
        actual_velocity[R] = SChassisIF::feedback[R]->actual_velocity / 360.f * DISPLACEMENT_PER_ROUND;
        actual_velocity[L] = SChassisIF::feedback[L]->actual_velocity / 360.f * DISPLACEMENT_PER_ROUND;

        if (mode == FORCED_RELAX_MODE) {
            target_current[R] = target_current[L] = 0;
        } else {
            target_velocity[R] = a2v_pid[R].calc(accumulated_displacement[R], target_location_);
            target_velocity[L] = a2v_pid[L].calc(accumulated_displacement[L], target_location_);
            target_current[R] = (int)v2i_pid[R].calc(actual_velocity[R], target_velocity[R]);
            target_current[L] = (int)v2i_pid[L].calc(actual_velocity[L], target_velocity[L]);
        }
        if (!motor_enabled) for (int &c : target_current) c = 0;

        *SChassisIF::target_current[R] = target_current[R];
        *SChassisIF::target_current[L] = target_current[R];
        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}

const Shell::Command SChassisSKD::shellCommands[] = {
        {"_c",              nullptr,                                        SChassisSKD::cmdInfo,           nullptr},
        {"_c_enable_fb",    "Channel/All Feedback{Disabled,Enabled}",       SChassisSKD::cmdEnableFeedback, nullptr},
        {"_c_pid",          "Channel is_a2v [kp] [ki] [kd] [i_limit] [out_limit]", SChassisSKD::cmdPID,            nullptr},
        {"_c_enable_motor", "All Motor{Disabled,Enabled}",                  SChassisSKD::cmdEnableMotor,    nullptr},
        {nullptr,           nullptr,                                        nullptr,                       nullptr}
};

DEF_SHELL_CMD_START(SChassisSKD::cmdInfo)
    Shell::printf("_c:Sentry_Chassis" ENDL);
    Shell::printf("_c/Right:Angle{Target,Actual} Velocity{Target,Actual} Current{Target,Actual}" ENDL);
    Shell::printf("_c/Left:Angle{Target,Actual} Velocity{Target,Actual} Current{Target,Actual}" ENDL);
    return true;
DEF_SHELL_CMD_END

static bool feedbackEnabled[SChassisSKD::MOTOR_COUNT + 1] = {false, false, false};

void SChassisSKD::cmdFeedback(void *) {
    for (int i = 0; i <= MOTOR_COUNT; i++) {
        if (feedbackEnabled[i]) {
            Shell::printf("_c%d %.2f %.2f %.2f %.2f %d %d" ENDL, i,
                          accumulated_displacement[i], target_location_,
                          actual_velocity[i], target_velocity[i],
                          SChassisIF::feedback[i]->actual_current, *SChassisIF::target_current[i]);
        }
    }
}

DEF_SHELL_CMD_START(SChassisSKD::cmdEnableFeedback)
    int id;
    bool enabled;
    if (!Shell::parseIDAndBool(argc, argv, 2, id, enabled)) return false;
    if (id == -1) {
        for (bool &e : feedbackEnabled) e = enabled;
    } else {
        feedbackEnabled[id] = enabled;
    }
    return true;
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(SChassisSKD::cmdEnableMotor)
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

DEF_SHELL_CMD_START(SChassisSKD::cmdPID)
    if (argc < 2) return false;

    unsigned id = Shell::atoi(argv[0]);
    bool is_a2v = Shell::atoi(argv[1]) == 0;
    if (id >= MOTOR_COUNT) return false;

    PIDController *pid = is_a2v ? a2v_pid : v2i_pid;
    if (argc == 2) {
        pid_params_t params = pid[id].get_parameters();
        Shell::printf("_c_pid %u %.2f %.2f %.2f %.2f %.2f" SHELL_NEWLINE_STR,
                      id, params.kp, params.ki, params.kd, params.i_limit, params.out_limit);
    } else if (argc == 7) {
        pid[id].change_parameters({Shell::atof(argv[2]),
                                Shell::atof(argv[3]),
                                Shell::atof(argv[4]),
                                Shell::atof(argv[5]),
                                Shell::atof(argv[6])});
    } else {
        return false;
    }

    return true;
DEF_SHELL_CMD_END
