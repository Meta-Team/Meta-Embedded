//
// Created by liuzikai on 2019-07-12.
//

#include "sentry_settings.h"

ShellCommand mainProgramCommands[] = {
        {"s_get_gimbal", gimbal_get_config},
        {"s_set_gimbal", gimbal_set_config},
        {"s_get_chassis", chassis_get_config},
        {"s_set_chassis", chassis_set_config},
        {"s_get_shoot", shoot_get_config},
        {"s_set_shoot", shoot_set_config},
        {nullptr,         nullptr}
};

void gimbal_get_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s_get_gimbal");
        return;
    }
    chprintf(chp, "!sg,%f,%f,%f,%f,%f,%f,%f,%f" SHELL_NEWLINE_STR,
             User::yaw_sensitivity[0], User::yaw_sensitivity[1], User::yaw_sensitivity[2],
             User::gimbal_pitch_max_angle, User::gimbal_pitch_min_angle,
             User::pitch_sensitivity[0], User::pitch_sensitivity[1],
             User::pitch_sensitivity[2]);
}

void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 8) {
        shellUsage(chp, "s_set_gimbal (8 arguments)");
        chprintf(chp, "!se" SHELL_NEWLINE_STR);
        return;
    }
    User::yaw_sensitivity[0] = Shell::atof(argv[0]);
    User::yaw_sensitivity[1] = Shell::atof(argv[1]);
    User::yaw_sensitivity[2] = Shell::atof(argv[2]);
    User::gimbal_pitch_max_angle = Shell::atof(argv[3]);
    User::gimbal_pitch_min_angle = Shell::atof(argv[4]);
    User::pitch_sensitivity[0] = Shell::atof(argv[5]);
    User::pitch_sensitivity[1] = Shell::atof(argv[6]);
    User::pitch_sensitivity[2] = Shell::atof(argv[7]);

    chprintf(chp, "!so" SHELL_NEWLINE_STR);
}

void chassis_get_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s_get_chassis");
        return;
    }
    chprintf(chp, "!sc,%f,%f,%f,%f,%f,%c" SHELL_NEWLINE_STR,
             User::chassis_v, 0, 0, 0, 0,' ');
}

void chassis_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 6) {
        shellUsage(chp, "s_set_chassis (6 arguments)");
        chprintf(chp, "!se" SHELL_NEWLINE_STR);
        return;
    }

    User::chassis_v = Shell::atof(argv[0]);

    chprintf(chp, "!so" SHELL_NEWLINE_STR);
}

void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s_get_shoot");
        return;
    }
    chprintf(chp, "!ss,%f,%f,%f,%f,%c" SHELL_NEWLINE_STR,
             User::shoot_launch_left_count, User::shoot_launch_right_count,
             User::shoot_launch_speed,
             User::shoot_common_duty_cycle,
             ' ');
}

void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 5) {
        shellUsage(chp, "s_set_shoot (5 arguments)");
        chprintf(chp, "!se" SHELL_NEWLINE_STR);
        return;
    }

    User::shoot_launch_left_count = Shell::atof(argv[0]);
    User::shoot_launch_right_count = Shell::atof(argv[1]);
    User::shoot_launch_speed = Shell::atof(argv[2]);
    User::shoot_common_duty_cycle = Shell::atof(argv[3]);

    chprintf(chp, "!so" SHELL_NEWLINE_STR);
}