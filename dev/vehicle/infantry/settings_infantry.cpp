//
// Created by liuzikai on 2019-07-12.
//

#include "settings_infantry.h"
#include "gimbal_scheduler.h"
#include "chassis_scheduler.h"
#include "ahrs.h"

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

#define MOTOR_COUNT  14
#define THREAD_FEEDBACK_PRIO  (LOWPRIO + 6)
unsigned const GIMBAL_FEEDBACK_INTERVAL = 25; // [ms]

bool feedback_enable[MOTOR_COUNT];

const char *motor_name[MOTOR_COUNT] = {
        "yaw",
        "pitch",
        "front_right",
        "front_left",
        "back_left",
        "back_right",
        "ahrs",
        "gyro",
        "accel",
        "magnet",
        "vision_armor",
        "vision_shoot",
        "vision_command"};

void cmd_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 1) {
        shellUsage(chp, "fb_enable motor_id");
        return;
    }
    int motor_id = Shell::atoi(argv[0]);
    if (motor_id < 0 || motor_id >= MOTOR_COUNT) {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return;
    }
    feedback_enable[motor_id] = true;
    chprintf(chp, "%s feedback enabled" SHELL_NEWLINE_STR, motor_name[motor_id]);
}

void cmd_disable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 1) {
        shellUsage(chp, "fb_disable motor_id");
        return;
    }
    int motor_id = Shell::atoi(argv[0]);
    if (motor_id < 0 || motor_id >= MOTOR_COUNT) {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return;
    }
    feedback_enable[motor_id] = false;
    chprintf(chp, "%s feedback disabled" SHELL_NEWLINE_STR, motor_name[motor_id]);
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

void cmd_set_pid(BaseSequentialStream *chp, int argc, char **argv) {
    if (argc != 7) {
        shellUsage(chp, "set_pid motor_id pid_id(0: angle_to_v, 1: v_to_i) ki kp kd i_limit out_limit");
        return;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    unsigned pid_id = Shell::atoi(argv[1]);
    PIDController::pid_params_t pid_param = {Shell::atof(argv[2]),
                                             Shell::atof(argv[3]),
                                             Shell::atof(argv[4]),
                                             Shell::atof(argv[5]),
                                             Shell::atof(argv[6])};
    if (motor_id < 2) {
        GimbalSKD::load_pid_params_by_type(pid_param, (GimbalBase::motor_id_t) motor_id, pid_id == 0);
    } else if (motor_id >= 2 && motor_id < 6) {
        ChassisSKD::load_pid_params_by_type(pid_param, pid_id == 0);
    } else {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
    }
}

void cmd_echo_param(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "echo_pid motor_id pid_id(0: angle_to_v, 1: v_to_i)");
        return;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    unsigned pid_id = Shell::atoi(argv[1]);

    if (motor_id < 6) {
        PIDController::pid_params_t pid_param = {0,0,0,0,0};
        if (motor_id < 2) {
            pid_param = GimbalSKD::echo_pid_params_by_type((GimbalBase::motor_id_t) motor_id, pid_id == 0);
        } else {
            pid_param = ChassisSKD::echo_pid_params_by_type(pid_id == 0);
        }
        chprintf(chp, "ki: %.2f, kp: %.2f, kd: %.2f, i_limit: %.2f, out_limit: %.2f" SHELL_NEWLINE_STR,
                 pid_param.ki, pid_param.kp, pid_param.kd, pid_param.i_limit, pid_param.out_limit);
    } else {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
    }
}

void cmd_vision_bullet_speed(BaseSequentialStream *chp, int argc, char **argv) {
    if (argc == 0) {
        chprintf(chp, "%f" SHELL_NEWLINE_STR, Vision::get_bullet_speed());
    } else if (argc == 1) {
        Vision::set_bullet_speed(Shell::atof(argv[0]));
    } else {
        shellUsage(chp, "vision_bullet_speed [setting]");
    }
}

// Shell commands to ...
ShellCommand mainProgramCommands[] = {
        {"fb_enable",  cmd_enable_feedback},
        {"fb_disable", cmd_disable_feedback},
        {"echo_param", cmd_echo_param},
        {"set_pid",             cmd_set_pid},
        {"vision_bullet_speed", cmd_vision_bullet_speed},
        {nullptr,               nullptr}
};

extern AHRSOnBoard ahrs;

class FeedbackThread : public BaseStaticThread<512> {
private:
    void main() final {
        setName("Feedback");
        while (!shouldTerminate()) {
            float actual_angle, target_angle, actual_velocity, target_velocity;

            // Gimbal
            for (int i = 0; i < 2; i++) {
                if (feedback_enable[i]) {
                    actual_angle = GimbalSKD::get_actual_angle((GimbalBase::motor_id_t) i);
                    target_angle = GimbalSKD::get_target_angle((GimbalBase::motor_id_t) i);
                    actual_velocity = GimbalSKD::get_actual_velocity((GimbalBase::motor_id_t) i);
                    target_velocity = GimbalSKD::get_target_velocity((GimbalBase::motor_id_t) i);
                    Shell::printf("fb %s %.2f %.2f %.2f %.2f %d %d" SHELL_NEWLINE_STR,
                                  motor_name[i],
                                  actual_angle, target_angle,
                                  actual_velocity, target_velocity,
                                  GimbalIF::feedback[i]->actual_current, *GimbalIF::target_current[i]);
                }
            }

            // Chassis
            for (int i = 2; i < 6; i++) {
                if (feedback_enable[i]) {
                    actual_angle = ChassisSKD::get_actual_theta();
                    target_angle = ChassisSKD::get_target_theta();
                    actual_velocity = ChassisSKD::get_actual_velocity((ChassisBase::motor_id_t) (i - 2));
                    target_velocity = ChassisSKD::get_target_velocity((ChassisBase::motor_id_t) (i - 2));
                    Shell::printf("fb %s %.2f %.2f %.2f %.2f %d %d" SHELL_NEWLINE_STR,
                                  motor_name[i],
                                  actual_angle, target_angle,
                                  actual_velocity, target_velocity,
                                  ChassisIF::feedback[i-2]->actual_current, *ChassisIF::target_current[i-2]);
                }
            }

            // AHRS
            if (feedback_enable[6])
                Shell::printf("fb %s %.2f 0 %.2f 0 %.2f 0" SHELL_NEWLINE_STR,
                              motor_name[6], ahrs.get_angle().x, ahrs.get_angle().y, ahrs.get_angle().z);
            if (feedback_enable[7])
                Shell::printf("fb %s %.2f 0 %.2f 0 %.2f 0" SHELL_NEWLINE_STR,
                              motor_name[7], ahrs.get_gyro().x, ahrs.get_gyro().y, ahrs.get_gyro().z);
            if (feedback_enable[8])
                Shell::printf("fb %s %.2f 0 %.2f 0 %.2f 0" SHELL_NEWLINE_STR,
                              motor_name[8], ahrs.get_accel().x, ahrs.get_accel().y, ahrs.get_accel().z);
            if (feedback_enable[9])
                Shell::printf("fb %s %.2f 0 %.2f 0 %.2f 0" SHELL_NEWLINE_STR,
                              motor_name[9], ahrs.get_magnet().x, ahrs.get_magnet().y, ahrs.get_magnet().z);

            // Vision
            if (feedback_enable[6]) {
                Shell::printf("fb %s %.2f %.2f %.2f %.2f %.2f %.2f" SHELL_NEWLINE_STR, motor_name[6],
                              Vision::armor_ypd[0].get_position(), Vision::armor_ypd[0].get_velocity() * 1000,
                              Vision::armor_ypd[1].get_position(), Vision::armor_ypd[1].get_velocity() * 1000,
                              Vision::armor_ypd[2].get_position(), Vision::armor_ypd[2].get_velocity() * 1000);
            }

            if (feedback_enable[7]) {
                /*Shell::printf("fb %s %d %d %d %d %.2f %d" SHELL_NEWLINE_STR, motor_name[7],
                              Vision::expected_shoot_after_periods * 10, // blue
                              (int) Vision::pak.command.period,  // red
                              Vision::bullet_flight_time,  // blue
                              (int) Vision::pak.command.remaining_time_to_target, //red
                              Vision::measured_shoot_delay.get(),  // blue
                              (int) Vision::last_update_time_delta  // red
                );*/
            }

            if (feedback_enable[8]) {
                Shell::printf("fb %s %.2f 0 %.2f 0 %.2f 0" SHELL_NEWLINE_STR, motor_name[8],
                              Vision::pak.command.yaw_delta / 100.0f, // blue
                              Vision::pak.command.pitch_delta / 100.0f, // blue
                              Vision::pak.command.dist / 100.0f // blue
                );
            }

            sleep(TIME_MS2I(GIMBAL_FEEDBACK_INTERVAL));
        }
    }
} feedbackThread;

void feedback_thread_start() {
    feedbackThread.start(THREAD_FEEDBACK_PRIO);
}
