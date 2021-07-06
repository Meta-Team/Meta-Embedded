//
// Created by liuzikai on 2019-07-12.
//

#include "settings_infantry.h"

using namespace chibios_rt;
//ShellCommand mainProgramCommands[] = {
//        {"s_get_gimbal", gimbal_get_config},
//        {"s_set_gimbal", gimbal_set_config},
//        {"s_get_chassis", chassis_get_config},
//        {"s_set_chassis", chassis_set_config},
//        {"s_get_shoot", shoot_get_config},
//        {"s_set_shoot", shoot_set_config},
//        {nullptr,         nullptr}
//};
//
//void gimbal_get_config(BaseSequentialStream *chp, int argc, char *argv[]) {
//    (void) argv;
//    if (argc != 0) {
//        shellUsage(chp, "s_get_gimbal");
//        return;
//    }
//    chprintf(chp, "!sg,%f,%f,%f,%f,%f,%f,%f,%f" SHELL_NEWLINE_STR,
//             UserI::gimbal_pc_yaw_sensitivity[0], UserI::gimbal_pc_yaw_sensitivity[1], UserI::gimbal_pc_yaw_sensitivity[2],
//             UserI::gimbal_pitch_max_angle, UserI::gimbal_pitch_min_angle,
//             UserI::gimbal_pc_pitch_sensitivity[0], UserI::gimbal_pc_pitch_sensitivity[1],
//             UserI::gimbal_pc_pitch_sensitivity[2]);
//}
//
//void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
//    if (argc != 8) {
//        shellUsage(chp, "s_set_gimbal (8 arguments)");
//        chprintf(chp, "!se" SHELL_NEWLINE_STR);
//        return;
//    }
//    UserI::gimbal_pc_yaw_sensitivity[0] = Shell::atof(argv[0]);
//    UserI::gimbal_pc_yaw_sensitivity[1] = Shell::atof(argv[1]);
//    UserI::gimbal_pc_yaw_sensitivity[2] = Shell::atof(argv[2]);
//    UserI::gimbal_pitch_max_angle = Shell::atof(argv[3]);
//    UserI::gimbal_pitch_min_angle = Shell::atof(argv[4]);
//    UserI::gimbal_pc_pitch_sensitivity[0] = Shell::atof(argv[5]);
//    UserI::gimbal_pc_pitch_sensitivity[1] = Shell::atof(argv[6]);
//    UserI::gimbal_pc_pitch_sensitivity[2] = Shell::atof(argv[7]);
//
//    chprintf(chp, "!so" SHELL_NEWLINE_STR);
//}
//
//void chassis_get_config(BaseSequentialStream *chp, int argc, char *argv[]) {
//    (void) argv;
//    if (argc != 0) {
//        shellUsage(chp, "s_get_chassis");
//        return;
//    }
//    chprintf(chp, "!sc,%f,%f,%f,%f,%f,%c" SHELL_NEWLINE_STR,
//             UserI::chassis_v_forward, UserI::chassis_v_backward, UserI::chassis_v_left_right,
//             UserI::chassis_pc_shift_ratio, UserI::chassis_pc_ctrl_ratio,
//             Remote::key2char(UserI::chassis_dodge_switch));
//}
//
//void chassis_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
//    if (argc != 6) {
//        shellUsage(chp, "s_set_chassis (6 arguments)");
//        chprintf(chp, "!se" SHELL_NEWLINE_STR);
//        return;
//    }
//
//    UserI::chassis_v_forward = Shell::atof(argv[0]);
//    UserI::chassis_v_backward = Shell::atof(argv[1]);
//    UserI::chassis_v_left_right = Shell::atof(argv[2]);
//    UserI::chassis_pc_shift_ratio = Shell::atof(argv[3]);
//    UserI::chassis_pc_ctrl_ratio = Shell::atof(argv[4]);
//    UserI::chassis_dodge_switch = Remote::char2key(argv[5][0]);
//
//    chprintf(chp, "!so" SHELL_NEWLINE_STR);
//}
//
//void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]) {
//    (void) argv;
//    if (argc != 0) {
//        shellUsage(chp, "s_get_shoot");
//        return;
//    }
//    chprintf(chp, "!ss,%f,%f,%f,%f,%c" SHELL_NEWLINE_STR,
//             UserI::shoot_launch_left_count, UserI::shoot_launch_right_count,
//             UserI::shoot_launch_speed,
//             UserI::shoot_common_duty_cycle,
//             Remote::key2char(UserI::shoot_fw_switch));
//}
//
//void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]) {
//    if (argc != 5) {
//        shellUsage(chp, "s_set_shoot (5 arguments)");
//        chprintf(chp, "!se" SHELL_NEWLINE_STR);
//        return;
//    }
//
//    UserI::shoot_launch_left_count = Shell::atof(argv[0]);
//    UserI::shoot_launch_right_count = Shell::atof(argv[1]);
//    UserI::shoot_launch_speed = Shell::atof(argv[2]);
//    UserI::shoot_common_duty_cycle = Shell::atof(argv[3]);
//    UserI::shoot_fw_switch = Remote::char2key(argv[4][0]);
//
//    chprintf(chp, "!so" SHELL_NEWLINE_STR);
//}

#define MOTOR_COUNT 2
#define THREAD_FEEDBACK_PRIO                (LOWPRIO + 6)
unsigned const GIMBAL_FEEDBACK_INTERVAL = 25; // [ms]

bool feedback_enable[MOTOR_COUNT];

const char *motor_name[] = {"yaw", "pitch"};

void cmd_t3_echo_motors(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 0) {
        shellUsage(chp, "t3_echo_motors");
        return;
    }

}

void cmd_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "fb_enable motor(yaw(0)/pitch(1)) set_enable(0/1)");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    int motor_id = Shell::atoi(argv[0]);
    if (motor_id < 0 || motor_id >= MOTOR_COUNT) {
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    feedback_enable[motor_id] = *argv[1] != '0';
    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

void cmd_set_target_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "set_a yaw_angle pitch_angle");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    GimbalLG::set_target(Shell::atof((argv[0])), Shell::atof(argv[1]));
    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}


//static void cmd_echo_status(BaseSequentialStream *chp, int argc, char *argv[]) {
//    (void) argv;
//    if (argc != 0) {
//        shellUsage(chp, "echo_status");
//        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
//        return;
//    }
//    chprintf(chp, "angle: %d %d %d" SHELL_NEWLINE_STR, MotorIF::feedback[YAW]->last_angle_raw, MotorIF::feedback[PITCH]->last_angle_raw, MotorIF::feedback[BULLET]->last_angle_raw);
//    chprintf(chp, "target_angle: %d %d %d" SHELL_NEWLINE_STR, targetAngle[YAW], targetAngle[PITCH], targetAngle[BULLET]);
//    chprintf(chp, "velocity: %.2f %.2f %.2f %.2f %.2f" SHELL_NEWLINE_STR, MotorIF::feedback[YAW]->actual_velocity, MotorIF::feedback[PITCH]->actual_velocity, MotorIF::feedback[BULLET]->actual_velocity, MotorIF::feedback[FW_LEFT]->actual_velocity, MotorIF::feedback[FW_RIGHT]->actual_velocity);
//    chprintf(chp, "target_velocity: %.2f %.2f %.2f %.2f %.2f" SHELL_NEWLINE_STR, targetVelocity[YAW], targetVelocity[PITCH], targetVelocity[BULLET], targetVelocity[FW_LEFT], targetVelocity[FW_RIGHT]);
//    chprintf(chp, "current: %d %d %d %d %d" SHELL_NEWLINE_STR, MotorIF::feedback[YAW]->actual_current, MotorIF::feedback[PITCH]->actual_current, MotorIF::feedback[BULLET]->actual_current, MotorIF::feedback[FW_LEFT]->actual_current, MotorIF::feedback[FW_RIGHT]->actual_current);
//    chprintf(chp, "target_current: %d %d %d %d %d" SHELL_NEWLINE_STR, targetCurrent[YAW], targetCurrent[PITCH], targetCurrent[BULLET], targetCurrent[FW_LEFT], targetCurrent[FW_RIGHT]);
//    chprintf(chp, "v2i_enable: %d %d %d %d %d" SHELL_NEWLINE_STR, v2i_enable[YAW], v2i_enable[PITCH], v2i_enable[BULLET], v2i_enable[FW_LEFT], v2i_enable[FW_RIGHT]);
//    chprintf(chp, "a2v_enable: %d %d %d %d %d" SHELL_NEWLINE_STR, a2v_enable[YAW], a2v_enable[PITCH], a2v_enable[BULLET], a2v_enable[FW_LEFT], a2v_enable[FW_RIGHT]);
//    chprintf(chp, "remote_enable: %d" SHELL_NEWLINE_STR, remote_enable);
//}

void cmd_set_param(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 7) {
        shellUsage(chp, "set_pid motor(yaw(0)/pitch(1)) pid_id(angle_to_v(0)/v_to_i(1)) ki kp kd i_limit out_limit");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    GimbalSKD::load_pid_params_by_id({Shell::atof(argv[2]),
                                      Shell::atof(argv[3]),
                                      Shell::atof(argv[4]),
                                      Shell::atof(argv[5]),
                                      Shell::atof(argv[6])}, *argv[0] == '0', *argv[1] == '0');

    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

// Shell commands to ...
ShellCommand mainProgramCommands[] = {
        {"fb_enable", cmd_enable_feedback},
        {"set_a",     cmd_set_target_angle},
        {"set_pid",   cmd_set_param},
        {nullptr,     nullptr}
};

class FeedbackThread : public BaseStaticThread<512> {
private:
    void main() final {
        setName("Feedback");
        while (!shouldTerminate()) {
            float actual_angle, target_angle, actual_velocity, target_velocity;
            for (int i = 0; i < MOTOR_COUNT; i++) {
                if (feedback_enable[i]) {
                    actual_angle = GimbalIF::feedback[i]->actual_angle;
                    target_angle = GimbalSKD::get_target_angle((GimbalBase::motor_id_t) i);
                    actual_velocity = GimbalIF::feedback[i]->actual_velocity;
                    target_velocity = GimbalSKD::get_target_velocity((GimbalBase::motor_id_t) i);
                    Shell::printf("fb %s %.2f %.2f %.2f %.2f %d %d" SHELL_NEWLINE_STR,
                                  motor_name[i],
                                  actual_angle, target_angle,
                                  actual_velocity, target_velocity,
                                  GimbalIF::feedback[i]->actual_current, GimbalIF::target_current[i]);
                }
            }
            sleep(TIME_MS2I(GIMBAL_FEEDBACK_INTERVAL));
        }
    }
} feedbackThread;

void feedback_thread_start() {
    feedbackThread.start(THREAD_FEEDBACK_PRIO);
}