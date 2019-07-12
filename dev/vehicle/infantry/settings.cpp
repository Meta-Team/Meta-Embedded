//
// Created by liuzikai on 2019-07-12.
//

#include "settings.h"

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
    chprintf(chp, "!sg,%f,%f,%f,%f,%f,%f,%f,%f",
             User::gimbal_pc_yaw_sensitivity[0], User::gimbal_pc_yaw_sensitivity[1], User::gimbal_pc_yaw_sensitivity[2],
             User::gimbal_pitch_max_angle, User::gimbal_pitch_min_angle,
             User::gimbal_pc_pitch_sensitivity[0], User::gimbal_pc_pitch_sensitivity[1],
             User::gimbal_pc_pitch_sensitivity[2]);
}

void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 8) {
        shellUsage(chp, "s_set_gimbal (8 arguments)");
        chprintf(chp, "!se");
        return;
    }
    User::gimbal_pc_yaw_sensitivity[0] = Shell::atof(argv[0]);
    User::gimbal_pc_yaw_sensitivity[1] = Shell::atof(argv[1]);
    User::gimbal_pc_yaw_sensitivity[2] = Shell::atof(argv[2]);
    User::gimbal_pitch_max_angle = Shell::atof(argv[3]);
    User::gimbal_pitch_min_angle = Shell::atof(argv[4]);
    User::gimbal_pc_pitch_sensitivity[0] = Shell::atof(argv[5]);
    User::gimbal_pc_pitch_sensitivity[1] = Shell::atof(argv[6]);
    User::gimbal_pc_pitch_sensitivity[2] = Shell::atof(argv[7]);

    chprintf(chp, "!so");
}

void chassis_get_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s_get_chassis");
        return;
    }
    chprintf(chp, "!sc,%f,%f,%f,%f,%f,%c",
             User::chassis_v_forward, User::chassis_v_backward, User::chassis_v_left_right,
             User::chassis_pc_shift_ratio, User::chassis_pc_ctrl_ratio,
             Remote::key2char(User::chassis_dodge_switch));
}

void chassis_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 6) {
        shellUsage(chp, "s_set_chassis (6 arguments)");
        chprintf(chp, "!se");
        return;
    }

    User::chassis_v_forward = Shell::atof(argv[0]);
    User::chassis_v_backward = Shell::atof(argv[1]);
    User::chassis_v_left_right = Shell::atof(argv[2]);
    User::chassis_pc_shift_ratio = Shell::atof(argv[3]);
    User::chassis_pc_ctrl_ratio = Shell::atof(argv[4]);
    User::chassis_dodge_switch = Remote::char2key(argv[5][0]);

    chprintf(chp, "!so");
}

void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s_get_shoot");
        return;
    }
    chprintf(chp, "!ss,%f,%f,%f,%f,%c",
             User::shoot_launch_left_count, User::shoot_launch_right_count,
             User::shoot_launch_speed,
             User::shoot_common_duty_cycle,
             Remote::key2char(User::shoot_fw_switch));
}

void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 5) {
        shellUsage(chp, "s_set_shoot (5 arguments)");
        chprintf(chp, "!se");
        return;
    }

    User::shoot_launch_left_count = Shell::atof(argv[0]);
    User::shoot_launch_right_count = Shell::atof(argv[1]);
    User::shoot_launch_speed = Shell::atof(argv[2]);
    User::shoot_common_duty_cycle = Shell::atof(argv[3]);
    User::shoot_fw_switch = Remote::char2key(Shell::atof(argv[4]));

    chprintf(chp, "!so");
}