//
// Created by liuzikai on 2019-07-12.
//

#include "settings_infantry.h"

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
             UserI::gimbal_pc_yaw_sensitivity[0], UserI::gimbal_pc_yaw_sensitivity[1], UserI::gimbal_pc_yaw_sensitivity[2],
             UserI::gimbal_pitch_max_angle, UserI::gimbal_pitch_min_angle,
             UserI::gimbal_pc_pitch_sensitivity[0], UserI::gimbal_pc_pitch_sensitivity[1],
             UserI::gimbal_pc_pitch_sensitivity[2]);
}

void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 8) {
        shellUsage(chp, "s_set_gimbal (8 arguments)");
        chprintf(chp, "!se" SHELL_NEWLINE_STR);
        return;
    }
    UserI::gimbal_pc_yaw_sensitivity[0] = Shell::atof(argv[0]);
    UserI::gimbal_pc_yaw_sensitivity[1] = Shell::atof(argv[1]);
    UserI::gimbal_pc_yaw_sensitivity[2] = Shell::atof(argv[2]);
    UserI::gimbal_pitch_max_angle = Shell::atof(argv[3]);
    UserI::gimbal_pitch_min_angle = Shell::atof(argv[4]);
    UserI::gimbal_pc_pitch_sensitivity[0] = Shell::atof(argv[5]);
    UserI::gimbal_pc_pitch_sensitivity[1] = Shell::atof(argv[6]);
    UserI::gimbal_pc_pitch_sensitivity[2] = Shell::atof(argv[7]);

    chprintf(chp, "!so" SHELL_NEWLINE_STR);
}

void chassis_get_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s_get_chassis");
        return;
    }
    chprintf(chp, "!sc,%f,%f,%f,%f,%f,%c" SHELL_NEWLINE_STR,
             UserI::chassis_v_forward, UserI::chassis_v_backward, UserI::chassis_v_left_right,
             UserI::chassis_pc_shift_ratio, UserI::chassis_pc_ctrl_ratio,
             Remote::key2char(UserI::chassis_dodge_switch));
}

void chassis_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 6) {
        shellUsage(chp, "s_set_chassis (6 arguments)");
        chprintf(chp, "!se" SHELL_NEWLINE_STR);
        return;
    }

    UserI::chassis_v_forward = Shell::atof(argv[0]);
    UserI::chassis_v_backward = Shell::atof(argv[1]);
    UserI::chassis_v_left_right = Shell::atof(argv[2]);
    UserI::chassis_pc_shift_ratio = Shell::atof(argv[3]);
    UserI::chassis_pc_ctrl_ratio = Shell::atof(argv[4]);
    UserI::chassis_dodge_switch = Remote::char2key(argv[5][0]);

    chprintf(chp, "!so" SHELL_NEWLINE_STR);
}

void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s_get_shoot");
        return;
    }
    chprintf(chp, "!ss,%f,%f,%f,%f,%c" SHELL_NEWLINE_STR,
             UserI::shoot_launch_left_count, UserI::shoot_launch_right_count,
             UserI::shoot_launch_speed,
             UserI::shoot_common_duty_cycle,
             Remote::key2char(UserI::shoot_fw_switch));
}

void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 5) {
        shellUsage(chp, "s_set_shoot (5 arguments)");
        chprintf(chp, "!se" SHELL_NEWLINE_STR);
        return;
    }

    UserI::shoot_launch_left_count = Shell::atof(argv[0]);
    UserI::shoot_launch_right_count = Shell::atof(argv[1]);
    UserI::shoot_launch_speed = Shell::atof(argv[2]);
    UserI::shoot_common_duty_cycle = Shell::atof(argv[3]);
    UserI::shoot_fw_switch = Remote::char2key(argv[4][0]);

    chprintf(chp, "!so" SHELL_NEWLINE_STR);
}