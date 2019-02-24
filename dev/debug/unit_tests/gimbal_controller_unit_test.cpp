//
// Created by liuzikai on 2019-01-07.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "gimbal_interface.h"
#include "gimbal_controller.h"
#include "mpu6500.h"

using namespace chibios_rt;

// Calculation interval for gimbal thread
int const gimbal_thread_interval = 10; // ms
int const gimbal_feedback_interval = 25; // ms
int const maximum_current = 4000; // mA

float const yaw_min_angle = -170; // degree
float const yaw_max_angle = 170; // degree
float const pitch_min_angle = -60; // degree
float const pitch_max_angle = 60; // degree
float const one_bullet_step = 40.0; // degree

float const yaw_max_speed = 600; // absolute maximum, degree/s
float const pitch_max_speed = 300; // absolute maximum, degree/s

bool enable_angle_to_v_pid = false;

float yaw_target_angle = 0.0;
float yaw_target_velocity = 0.0;
float pitch_target_angle = 0.0;
float pitch_target_velocity = 0.0;
//float bullet_loader_target_angle = 0.0;
float bullet_loader_target_velocity = 0.0;

class GimbalFeedbackThread : public chibios_rt::BaseStaticThread<512> {

public:

    GimbalFeedbackThread() {
        enable_yaw_feedback = false;
        enable_pitch_feedback = false;
        enable_bullet_loader_feedback = false;
    }

    bool enable_yaw_feedback;
    bool enable_pitch_feedback;
    bool enable_bullet_loader_feedback;

private:

    void main() final {

        setName("gimbal_fb");

        while (!shouldTerminate()) {

            if (enable_yaw_feedback) {
                Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              TIME_I2MS(chibios_rt::System::getTime()),
                              GimbalInterface::yaw.actual_angle, yaw_target_angle,
                              GimbalInterface::yaw.angular_velocity, yaw_target_velocity,
                              GimbalInterface::yaw.actual_current, GimbalInterface::yaw.target_current);
//        Shell::printf("yaw round = %d" SHELL_NEWLINE_STR,
//                      GimbalInterface::yaw.round_count);
            }

            if (enable_pitch_feedback) {
                Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              TIME_I2MS(chibios_rt::System::getTime()),
                              GimbalInterface::pitch.actual_angle, pitch_target_angle,
                              GimbalInterface::pitch.angular_velocity, pitch_target_velocity,
                              GimbalInterface::pitch.actual_current, GimbalInterface::pitch.target_current);
//        Shell::printf("pitch round = %d" SHELL_NEWLINE_STR,
//                      GimbalInterface::pitch.round_count);
            }

            if (enable_bullet_loader_feedback) {
                // Bullet loader has no current feedback
                Shell::printf("!gp,%u,%.2f,%.2f,%d" SHELL_NEWLINE_STR,
                              TIME_I2MS(chibios_rt::System::getTime()),
                              GimbalInterface::bullet_loader.angular_velocity, bullet_loader_target_velocity,
                              GimbalInterface::bullet_loader.target_current);
            }

            sleep(TIME_MS2I(gimbal_feedback_interval));
        }

    }

} gimbalFeedbackThread;

int shooting_speed_mode = GimbalController::STOP;

CANInterface can1(&CAND1);


/**
 * @brief set enabled states of yaw and pitch motors
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1') ||
        (*argv[2] != '0' && *argv[2] != '1')) {
        shellUsage(chp, "g_enable yaw(0/1) pitch(0/1) bullet_loader(0/1)");
        return;
    }
    GimbalInterface::yaw.enabled = *argv[0] - '0';
    GimbalInterface::pitch.enabled = *argv[1] - '0';
    GimbalInterface::bullet_loader.enabled = *argv[2] - '0';

//    chprintf(chp, "Gimbal yaw enabled = %d" SHELL_NEWLINE_STR, GimbalInterface::yaw.enabled);
//    chprintf(chp, "Gimbal pitch enabled = %d" SHELL_NEWLINE_STR, GimbalInterface::pitch.enabled);
//    chprintf(chp, "Enabled set." SHELL_NEWLINE_STR);
}

/**
 * @brief set enabled state of friction wheels
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_enable_fw(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1 || (*argv[0] != '0' && *argv[0] != '1')) {
        shellUsage(chp, "g_enable_fw 0/1");
        return;
    }
    shooting_speed_mode = GimbalController::MIDDLE;
    GimbalInterface::friction_wheels.duty_cycle = GimbalController::shoot_duty_cycles[shooting_speed_mode];
    GimbalInterface::friction_wheels.enabled = *argv[0] - '0';

//    chprintf(chp, "Gimbal friction_wheels enabled = %d" SHELL_NEWLINE_STR, GimbalInterface::friction_wheels.enabled);

}

/**
 * @brief set feedback enable states
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1') ||
        (*argv[2] != '0' && *argv[2] != '1')) {
        shellUsage(chp, "g_enable_fb yaw(0/1) pitch(0/1) bullet_loader(0/1)");
        return;
    }
    gimbalFeedbackThread.enable_yaw_feedback = *argv[0] - '0';
    gimbalFeedbackThread.enable_pitch_feedback = *argv[1] - '0';
    gimbalFeedbackThread.enable_bullet_loader_feedback = *argv[2] - '0';

//    chprintf(chp, "Gimbal yaw feedback = %d" SHELL_NEWLINE_STR, feedbackModule.enable_yaw_feedback);
//    chprintf(chp, "Gimbal pitch feedback = %d" SHELL_NEWLINE_STR, feedbackModule.enable_pitch_feedback);
//    chprintf(chp, "Feedback set." SHELL_NEWLINE_STR);
}


/**
 * @brief set front_angle_raw with current actual angle
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_fix_front_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_fix");
        return;
    }
    GimbalInterface::yaw.reset_front_angle();
    GimbalInterface::pitch.reset_front_angle();
    GimbalInterface::bullet_loader.reset_front_angle();

//    chprintf(chp, "!f" SHELL_NEWLINE_STR);
}

/**
 * @brief set target velocity of yaw and pitch and disable pos_to_v_pid
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_set_target_velocities(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3) {
        shellUsage(chp, "g_set_v yaw_velocity pitch_velocity bullet_loader_velocity");
        return;
    }

    yaw_target_velocity = Shell::atof(argv[0]);
    pitch_target_velocity = Shell::atof(argv[1]);
    bullet_loader_target_velocity = Shell::atof(argv[2]);
    GimbalController::yaw.v_to_i_pid.clear_i_out();
    GimbalController::yaw.angle_to_v_pid.clear_i_out();
    GimbalController::pitch.v_to_i_pid.clear_i_out();
    GimbalController::pitch.angle_to_v_pid.clear_i_out();
    GimbalController::bullet_loader.v_to_i_pid.clear_i_out();
//    chprintf(chp, "Gimbal yaw target_velocity = %f" SHELL_NEWLINE_STR, yaw_target_velocity);
//    chprintf(chp, "Gimbal pitch target_velocity = %f" SHELL_NEWLINE_STR, pitch_target_velocity);

    enable_angle_to_v_pid = false;
//    chprintf(chp, "Target velocity set. pos_to_v_pid disabled." SHELL_NEWLINE_STR);
}

/**
 * @brief set target angle of yaw and pitch and enable pos_to_v_pid
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_set_target_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_angle yaw_angle pitch_angle");
        return;
    }

    yaw_target_angle = Shell::atof(argv[0]);
    pitch_target_angle = Shell::atof(argv[1]);
    GimbalController::yaw.v_to_i_pid.clear_i_out();
    GimbalController::yaw.angle_to_v_pid.clear_i_out();
    GimbalController::pitch.v_to_i_pid.clear_i_out();
    GimbalController::pitch.angle_to_v_pid.clear_i_out();
    GimbalController::bullet_loader.v_to_i_pid.clear_i_out();
//    chprintf(chp, "Gimbal yaw target_angle = %f" SHELL_NEWLINE_STR, yaw_target_angle);
//    chprintf(chp, "Gimbal pitch target_angle = %f" SHELL_NEWLINE_STR, pitch_target_angle);

    enable_angle_to_v_pid = true;
//    chprintf(chp, "Target angle set. pos_to_v_pid enabled." SHELL_NEWLINE_STR);
}

/**
 * @brief set pid parameters
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 7) {
        shellUsage(chp,
                   "g_set_params yaw(0)/pitch(1)/bullet_loader(2) angle_to_v(0)/v_to_i(0) ki kp kd i_limit out_limit");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    PIDController *pidController = nullptr;
    if (*argv[0] == '0' && *argv[1] == '0') pidController = &GimbalController::yaw.angle_to_v_pid;
    else if (*argv[0] == '0' && *argv[1] == '1') pidController = &GimbalController::yaw.v_to_i_pid;
    else if (*argv[0] == '1' && *argv[1] == '0') pidController = &GimbalController::pitch.angle_to_v_pid;
    else if (*argv[0] == '1' && *argv[1] == '1') pidController = &GimbalController::pitch.v_to_i_pid;
    else if (*argv[0] == '2' && *argv[1] == '1') pidController = &GimbalController::bullet_loader.v_to_i_pid;
    else {
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    pidController->change_parameters(Shell::atof(argv[2]),
                                     Shell::atof(argv[3]),
                                     Shell::atof(argv[4]),
                                     Shell::atof(argv[5]),
                                     Shell::atof(argv[6]));
    GimbalController::yaw.v_to_i_pid.clear_i_out();
    GimbalController::yaw.angle_to_v_pid.clear_i_out();
    GimbalController::pitch.v_to_i_pid.clear_i_out();
    GimbalController::pitch.angle_to_v_pid.clear_i_out();
    GimbalController::bullet_loader.v_to_i_pid.clear_i_out();
    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

/**
 * @brief helper function for cmd_gimbal_echo_parameters()
 * @param chp
 * @param pidController
 */
static inline void _cmd_gimbal_echo_parameters(BaseSequentialStream *chp, PIDController *pidController) {
    chprintf(chp, "%f %f %f %f %f" SHELL_NEWLINE_STR,
             pidController->kp,
             pidController->ki,
             pidController->kd,
             pidController->i_limit,
             pidController->out_limit);
}

/**
 * @brief echo pid parameters
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_echo_params");
        return;
    }

    chprintf(chp, "yaw angle_to_v:   ");
    _cmd_gimbal_echo_parameters(chp, &GimbalController::yaw.angle_to_v_pid);
    chprintf(chp, "yaw v_to_i:       ");
    _cmd_gimbal_echo_parameters(chp, &GimbalController::yaw.v_to_i_pid);
    chprintf(chp, "pitch angle_to_v: ");
    _cmd_gimbal_echo_parameters(chp, &GimbalController::pitch.angle_to_v_pid);
    chprintf(chp, "pitch v_to_i:     ");
    _cmd_gimbal_echo_parameters(chp, &GimbalController::pitch.v_to_i_pid);
    chprintf(chp, "bullet_loader v_to_i:     ");
    _cmd_gimbal_echo_parameters(chp, &GimbalController::bullet_loader.v_to_i_pid);
}

static void cmd_gimbal_continuous_shooting(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_continuous_shooting");
        return;
    }
    GimbalController::bullet_loader.start_continuous_shooting();
}

static void cmd_gimbal_incontinuous_shooting(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "g_incontinuous_shooting bullet_number");
        return;
    }
    int bullet_num = Shell::atoi(argv[0]);
    if (bullet_num > 0) GimbalController::bullet_loader.start_incontinuous_shooting(bullet_num);
}

static void cmd_gimbal_stop_shooting(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_stop_shooting");
        return;
    }
    GimbalController::bullet_loader.stop_shooting();
    chprintf(chp, "remained bullets: %d" SHELL_NEWLINE_STR, GimbalController::bullet_loader.get_remained_bullet());
}

static void cmd_gimbal_set_shooting_speed(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 1) {
        shellUsage(chp, "g_set_shooting_speed duty_cycle");
        return;
    }

    GimbalInterface::friction_wheels.duty_cycle = Shell::atof(argv[0]);
}

static void cmd_gimbal_check(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        return;
    }
    chprintf(chp, "bullet_loader_velocity: %f" SHELL_NEWLINE_STR, GimbalInterface::bullet_loader.angular_velocity);
    chprintf(chp, "bullet_loader_target_velocity: %f" SHELL_NEWLINE_STR, bullet_loader_target_velocity);
    chprintf(chp, "bullet_loader_target_current: %d" SHELL_NEWLINE_STR, GimbalInterface::bullet_loader.target_current);
}

// Command lists for gimbal controller test and adjustments
ShellCommand gimbalCotrollerCommands[] = {
        {"g_enable",                cmd_gimbal_enable},
        {"g_enable_fb",             cmd_gimbal_enable_feedback},
        {"g_fix",                   cmd_gimbal_fix_front_angle},
        {"g_set_v",                 cmd_gimbal_set_target_velocities},
        {"g_set_angle",             cmd_gimbal_set_target_angle},
        {"g_set_params",            cmd_gimbal_set_parameters},
        {"g_echo_params",           cmd_gimbal_echo_parameters},
        {"g_enable_fw",             cmd_gimbal_enable_fw},
        {"g_continuous_shooting",   cmd_gimbal_continuous_shooting},
        {"g_incontinuous_shooting", cmd_gimbal_incontinuous_shooting},
        {"g_stop_shooting",         cmd_gimbal_stop_shooting},
        {"g_set_shooting_speed",    cmd_gimbal_set_shooting_speed},
        {"g_check",                 cmd_gimbal_check},
        {nullptr,                   nullptr}
};

/**
 * Main calculation thread
 */
class GimbalThread : public BaseStaticThread<256> {
protected:
    void main() final {
        setName("gimbal");
        while (!shouldTerminate()) {
//            Shell::printf("%f" SHELL_NEWLINE_STR, GimbalInterface::bullet_loader.angular_velocity);
            if (GimbalInterface::yaw.enabled || GimbalInterface::pitch.enabled ||
                GimbalInterface::bullet_loader.enabled) {


                // Calculate target velocity
                if (enable_angle_to_v_pid) {
                    yaw_target_velocity = GimbalController::yaw.angle_to_v(GimbalInterface::yaw.actual_angle,
                                                                           yaw_target_angle);
                    pitch_target_velocity = GimbalController::pitch.angle_to_v(GimbalInterface::pitch.actual_angle,
                                                                               pitch_target_angle);
                }

                // Calculate target current
                GimbalInterface::yaw.target_current = (int) GimbalController::yaw.v_to_i(
                        MPU6500Controller::angle_speed.z, yaw_target_velocity);
                GimbalInterface::pitch.target_current = (int) GimbalController::pitch.v_to_i(
                        MPU6500Controller::angle_speed.y, pitch_target_velocity);

                GimbalInterface::bullet_loader.target_current = (int) GimbalController::bullet_loader.get_target_current(
                        GimbalInterface::bullet_loader.angular_velocity, bullet_loader_target_velocity);


            }

            GimbalInterface::send_gimbal_currents();

            sleep(TIME_MS2I(gimbal_thread_interval));
        }
    }
} gimbalThread;


int main(void) {
    halInit();
    System::init();
    LED::red_off();
    LED::green_off();

    Shell::start(HIGHPRIO);
    Shell::addCommands(gimbalCotrollerCommands);

    MPU6500Controller::start(HIGHPRIO - 3);

    gimbalFeedbackThread.start(NORMALPRIO);

    can1.start(HIGHPRIO - 1);

    GimbalInterface::init(&can1);

    gimbalThread.start(HIGHPRIO - 2);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled,
    // main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}