//
// Created by zhukerui on 2019/6/8.
//

#include <control/suspension_gimbal_controller.h>
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "suspension_gimbal_interface.h"
#include "suspension_gimbal_controller.h"
#include "mpu6500.h"

using namespace chibios_rt;

// Calculation interval for gimbal thread
int const gimbal_thread_interval = 10; // ms
int const gimbal_feedback_interval = 25; // ms

float const yaw_min_angle = -170; // degree
float const yaw_max_angle = 170; // degree
float const pitch_min_angle = -60; // degree
float const pitch_max_angle = 60; // degree

float const yaw_max_speed = 600; // absolute maximum, degree/s
float const pitch_max_speed = 300; // absolute maximum, degree/s

bool enable_angle_to_v_pid = false;
bool fix_front = false;

float yaw_target_velocity = 0.0;
float pitch_target_velocity = 0.0;
float gimbal_fix_front_angle = 0.0;

int shooting_speed_mode = SuspensionGimbalIF::OFF;

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
                Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d" SHELL_NEWLINE_STR,
                              TIME_I2MS(chibios_rt::System::getTime()),
                              SuspensionGimbalIF::yaw.get_angular_position(), SuspensionGimbalController::target_yaw_angle,
                              SuspensionGimbalIF::yaw.angular_velocity, yaw_target_velocity,SuspensionGimbalIF::yaw.get_target_signal());
            }

            if (enable_pitch_feedback) {
                Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%d" SHELL_NEWLINE_STR,
                              TIME_I2MS(chibios_rt::System::getTime()),
                              SuspensionGimbalIF::pitch.get_angular_position(), SuspensionGimbalController::target_pitch_angle,
                              SuspensionGimbalIF::pitch.angular_velocity, pitch_target_velocity,SuspensionGimbalIF::pitch.get_target_signal());
            }

            if (enable_bullet_loader_feedback) {
                // Bullet loader has no current feedback
                Shell::printf("!gp,%u,%.2f,%d" SHELL_NEWLINE_STR,
                              TIME_I2MS(chibios_rt::System::getTime()),
                              SuspensionGimbalIF::bullet_loader.angular_velocity,
                              SuspensionGimbalIF::bullet_loader.get_target_signal());
            }

            sleep(TIME_MS2I(gimbal_feedback_interval));
        }

    }

} gimbalFeedbackThread;

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
    SuspensionGimbalController::set_motor_enable(SuspensionGimbalIF::YAW_ID, *argv[0] - '0');
    SuspensionGimbalController::set_motor_enable(SuspensionGimbalIF::PIT_ID, *argv[1] - '0');
    SuspensionGimbalController::set_motor_enable(SuspensionGimbalIF::BULLET_LOADER_ID, *argv[2] - '0');
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

    if(*argv[0] - '0'){
        SuspensionGimbalController::set_shoot_mode(SuspensionGimbalIF::AWAIT);
    } else{
        SuspensionGimbalController::set_shoot_mode(SuspensionGimbalIF::OFF);
    }

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
}

/**
 * @brief set front_angle_raw with current actual angle
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_set_front_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_front");
        return;
    }
    SuspensionGimbalIF::yaw.reset_front_angle();
    SuspensionGimbalIF::pitch.reset_front_angle();
    SuspensionGimbalIF::bullet_loader.reset_front_angle();
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
    SuspensionGimbalController::bullet_loader_speed = Shell::atof(argv[2]);

    SuspensionGimbalController::yaw_v_to_i.clear_i_out();
    SuspensionGimbalController::yaw_angle_to_v.clear_i_out();
    SuspensionGimbalController::pitch_v_to_i.clear_i_out();
    SuspensionGimbalController::pitch_angle_to_v.clear_i_out();
    SuspensionGimbalController::BL_v_to_i.clear_i_out();

    enable_angle_to_v_pid = false;
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
    if(fix_front){
        shellUsage(chp, "now is at the 'fix front mode' and the yaw won't be changed according to your command");
    }else{
        SuspensionGimbalController::target_yaw_angle = Shell::atof(argv[0]);
    }

    SuspensionGimbalController::target_pitch_angle = Shell::atof(argv[1]);

    SuspensionGimbalController::yaw_v_to_i.clear_i_out();
    SuspensionGimbalController::yaw_angle_to_v.clear_i_out();
    SuspensionGimbalController::pitch_v_to_i.clear_i_out();
    SuspensionGimbalController::pitch_angle_to_v.clear_i_out();
    SuspensionGimbalController::BL_v_to_i.clear_i_out();

    enable_angle_to_v_pid = true;
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
    if (*argv[0] == '0' && *argv[1] == '0') pidController = &SuspensionGimbalController::yaw_angle_to_v;
    else if (*argv[0] == '0' && *argv[1] == '1') pidController = &SuspensionGimbalController::yaw_v_to_i;
    else if (*argv[0] == '1' && *argv[1] == '0') pidController = &SuspensionGimbalController::pitch_angle_to_v;
    else if (*argv[0] == '1' && *argv[1] == '1') pidController = &SuspensionGimbalController::pitch_v_to_i;
    else if (*argv[0] == '2' && *argv[1] == '1') pidController = &SuspensionGimbalController::BL_v_to_i;
    else {
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    pidController->change_parameters(Shell::atof(argv[2]),
                                     Shell::atof(argv[3]),
                                     Shell::atof(argv[4]),
                                     Shell::atof(argv[5]),
                                     Shell::atof(argv[6]));
    SuspensionGimbalController::yaw_v_to_i.clear_i_out();
    SuspensionGimbalController::yaw_angle_to_v.clear_i_out();
    SuspensionGimbalController::pitch_v_to_i.clear_i_out();
    SuspensionGimbalController::pitch_angle_to_v.clear_i_out();
    SuspensionGimbalController::BL_v_to_i.clear_i_out();
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
    _cmd_gimbal_echo_parameters(chp, &SuspensionGimbalController::yaw_angle_to_v);
    chprintf(chp, "yaw v_to_i:       ");
    _cmd_gimbal_echo_parameters(chp, &SuspensionGimbalController::yaw_v_to_i);
    chprintf(chp, "pitch angle_to_v: ");
    _cmd_gimbal_echo_parameters(chp, &SuspensionGimbalController::pitch_angle_to_v);
    chprintf(chp, "pitch v_to_i:     ");
    _cmd_gimbal_echo_parameters(chp, &SuspensionGimbalController::pitch_v_to_i);
    chprintf(chp, "bullet_loader v_to_i:     ");
    _cmd_gimbal_echo_parameters(chp, &SuspensionGimbalController::BL_v_to_i);
}

static void cmd_gimbal_continuous_shooting(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "g_continuous_shoot 0/1");
        return;
    }
    if (*argv[0] - '0'){
        SuspensionGimbalController::start_continuous_shooting();
    } else{
        SuspensionGimbalController::stop_continuous_shooting();
    }
}

static void cmd_gimbal_incontinuous_shooting(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "g_incontinuous_shooting bullet_number");
        return;
    }
    int bullet_num = Shell::atoi(argv[0]);
    if (bullet_num > 0) SuspensionGimbalController::start_incontinuous_shooting(bullet_num);
}

static void cmd_gimbal_check(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        return;
    }
    chprintf(chp, "bullet_loader_velocity: %f" SHELL_NEWLINE_STR, SuspensionGimbalIF::bullet_loader.angular_velocity);
    chprintf(chp, "bullet_loader_target_current: %d" SHELL_NEWLINE_STR, SuspensionGimbalIF::bullet_loader.get_target_signal());
}

static void cmd_fix_front(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1){
        shellUsage(chp, "g_fix 0/1");
        return;
    }
    fix_front = *argv[0] - '0';
    if (fix_front){
        SuspensionGimbalIF::yaw.reset_front_angle();
        SuspensionGimbalController::target_yaw_angle = 0.0;
        chprintf(chp, "the 'fix front mode' is activated");
    } else{
        chprintf(chp, "the fix front mode is canceled");
    }
}

// Command lists for gimbal controller test and adjustments
ShellCommand gimbalCotrollerCommands[] = {
        {"g_enable",                cmd_gimbal_enable},
        {"g_enable_fb",             cmd_gimbal_enable_feedback},
        {"g_front",                   cmd_gimbal_set_front_angle},
        {"g_set_v",                 cmd_gimbal_set_target_velocities},
        {"g_set_angle",             cmd_gimbal_set_target_angle},
        {"g_set_params",            cmd_gimbal_set_parameters},
        {"g_echo_params",           cmd_gimbal_echo_parameters},
        {"g_enable_fw",             cmd_gimbal_enable_fw},
        {"g_continuous_shoot",   cmd_gimbal_continuous_shooting},
        {"g_incontinuous_shoot", cmd_gimbal_incontinuous_shooting},
        {"g_check",                 cmd_gimbal_check},
        {"g_fix",             cmd_fix_front},
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
            if (SuspensionGimbalIF::yaw.status() || SuspensionGimbalIF::pitch.status() ||
                    SuspensionGimbalIF::bullet_loader.status()) {
                // Calculate target signal
                if (enable_angle_to_v_pid){
                    SuspensionGimbalController::set_target_signal();
                } else{
                    SuspensionGimbalController::set_target_signal(SuspensionGimbalIF::YAW_ID,
                            (int)SuspensionGimbalController::yaw_v_to_i.calc(SuspensionGimbalIF::yaw.angular_velocity, yaw_target_velocity));
                    SuspensionGimbalController::set_target_signal(SuspensionGimbalIF::PIT_ID,
                            (int)SuspensionGimbalController::pitch_v_to_i.calc(SuspensionGimbalIF::pitch.angular_velocity, pitch_target_velocity));
                    SuspensionGimbalController::set_target_signal(SuspensionGimbalIF::BULLET_LOADER_ID,
                            (int)SuspensionGimbalController::BL_v_to_i.calc(SuspensionGimbalIF::bullet_loader.angular_velocity, SuspensionGimbalController::bullet_loader_speed));
                }
            }

            SuspensionGimbalIF::send_gimbal_currents();

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

    SuspensionGimbalIF::init(&can1);

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