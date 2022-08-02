//
// Created by Kerui Zhu on 7/19/2019.
//

#include "settings_aerial.h"

ShellCommand mainProgramCommands[] = {
        {"s_get_gimbal", gimbal_get_config},
        {"s_set_gimbal", gimbal_set_config},
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
             UserA::gimbal_pc_yaw_sensitivity[0], UserA::gimbal_pc_yaw_sensitivity[1], UserA::gimbal_pc_yaw_sensitivity[2],
             0, 0,
             UserA::gimbal_pc_pitch_sensitivity[0], UserA::gimbal_pc_pitch_sensitivity[1],
             UserA::gimbal_pc_pitch_sensitivity[2]);
}

void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 8) {
        shellUsage(chp, "s_set_gimbal (8 arguments)");
        chprintf(chp, "!se" SHELL_NEWLINE_STR);
        return;
    }
    UserA::gimbal_pc_yaw_sensitivity[0] = Shell::atof(argv[0]);
    UserA::gimbal_pc_yaw_sensitivity[1] = Shell::atof(argv[1]);
    UserA::gimbal_pc_yaw_sensitivity[2] = Shell::atof(argv[2]);
//    UserA::gimbal_pitch_max_angle = Shell::atof(argv[3]);
//    UserA::gimbal_pitch_min_angle = Shell::atof(argv[4]);
    UserA::gimbal_pc_pitch_sensitivity[0] = Shell::atof(argv[5]);
    UserA::gimbal_pc_pitch_sensitivity[1] = Shell::atof(argv[6]);
    UserA::gimbal_pc_pitch_sensitivity[2] = Shell::atof(argv[7]);

    chprintf(chp, "!so" SHELL_NEWLINE_STR);
}

void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s_get_shoot");
        return;
    }
    chprintf(chp, "!ss,%f,%f,%f,%f,%c" SHELL_NEWLINE_STR,
             UserA::shoot_launch_left_count, UserA::shoot_launch_right_count,
             UserA::shoot_launch_speed,
             UserA::shoot_common_duty_cycle,
             Remote::key2char(UserA::shoot_fw_switch));
}

void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 5) {
        shellUsage(chp, "s_set_shoot (5 arguments)");
        chprintf(chp, "!se" SHELL_NEWLINE_STR);
        return;
    }

    UserA::shoot_launch_left_count = Shell::atof(argv[0]);
    UserA::shoot_launch_right_count = Shell::atof(argv[1]);
    UserA::shoot_launch_speed = Shell::atof(argv[2]);
    UserA::shoot_common_duty_cycle = Shell::atof(argv[3]);
    UserA::shoot_fw_switch = Remote::char2key(argv[4][0]);

    chprintf(chp, "!so" SHELL_NEWLINE_STR);
}