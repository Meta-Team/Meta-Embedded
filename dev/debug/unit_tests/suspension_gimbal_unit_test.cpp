//
// Created by zhukerui on 2019/6/8.
//

#include <control/suspension_gimbal_skd.h>
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"
#include "can_interface.h"
#include "suspension_gimbal_interface.h"
#include "suspension_gimbal_skd.h"
#include "board.h"
//#include "mpu6500.h"

using namespace chibios_rt;

// Calculation interval for gimbal thread
int const gimbal_thread_interval = 10; // ms
int const gimbal_feedback_interval = 25; // ms
float target_yaw_angle = 0;
float target_pitch_angle = 0;

bool enable_angle_to_v_pid = false;
bool fix_front = false;

float yaw_target_velocity = 0.0;
float pitch_target_velocity = 0.0;

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
                              SuspensionGimbalIF::yaw.angular_position, target_yaw_angle,
                              SuspensionGimbalIF::yaw.angular_velocity, yaw_target_velocity,SuspensionGimbalIF::yaw.target_signal);
            }

            if (enable_pitch_feedback) {
                Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%d" SHELL_NEWLINE_STR,
                              TIME_I2MS(chibios_rt::System::getTime()),
                              SuspensionGimbalIF::pitch.angular_position, target_pitch_angle,
                              SuspensionGimbalIF::pitch.angular_velocity, pitch_target_velocity,SuspensionGimbalIF::pitch.target_signal);
            }

            if (enable_bullet_loader_feedback) {
                // Bullet loader has no current feedback
                Shell::printf("!gp,%u,%.2f,%d,%0.2f" SHELL_NEWLINE_STR,
                              TIME_I2MS(chibios_rt::System::getTime()),
                              SuspensionGimbalIF::bullet_loader.angular_velocity,
                              SuspensionGimbalIF::bullet_loader.target_signal,
                              SuspensionGimbalIF::bullet_loader.angular_position);
            }

            sleep(TIME_MS2I(gimbal_feedback_interval));
        }

    }

} gimbalFeedbackThread;

CANInterface can1(&CAND1);
AHRSExt ahrsExt;


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
    SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::YAW_ID, *argv[0] - '0');
    SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::PIT_ID, *argv[1] - '0');
    SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::BULLET_LOADER_ID, *argv[2] - '0');
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
        SuspensionGimbalSKD::set_shoot_mode(AWAIT);
    } else{
        SuspensionGimbalSKD::set_shoot_mode(OFF);
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
    SuspensionGimbalSKD::set_front(SuspensionGimbalIF::YAW_ID);
    SuspensionGimbalSKD::set_front(SuspensionGimbalIF::PIT_ID);
    SuspensionGimbalSKD::set_front(SuspensionGimbalIF::BULLET_LOADER_ID);
}

/**
 * @brief set target velocity of yaw and pitch and disable pos_to_v_pid
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_set_target_velocities(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_v yaw_velocity pitch_velocity");
        return;
    }

    yaw_target_velocity = Shell::atof(argv[0]);
    pitch_target_velocity = Shell::atof(argv[1]);

    SuspensionGimbalSKD::yaw_v2i_pid.clear_i_out();
    SuspensionGimbalSKD::yaw_a2v_pid.clear_i_out();
    SuspensionGimbalSKD::pitch_v2i_pid.clear_i_out();
    SuspensionGimbalSKD::pitch_a2v_pid.clear_i_out();
    SuspensionGimbalSKD::BL_v2i_pid.clear_i_out();

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
        SuspensionGimbalSKD::set_motor_angle(SuspensionGimbalIF::YAW_ID, Shell::atof(argv[0]));
    }

    SuspensionGimbalSKD::set_motor_angle(SuspensionGimbalIF::PIT_ID, Shell::atof(argv[1]));

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
    if (*argv[0] == '0' && *argv[1] == '0') pidController = &SuspensionGimbalSKD::yaw_a2v_pid;
    else if (*argv[0] == '0' && *argv[1] == '1') pidController = &SuspensionGimbalSKD::yaw_v2i_pid;
    else if (*argv[0] == '1' && *argv[1] == '0') pidController = &SuspensionGimbalSKD::pitch_a2v_pid;
    else if (*argv[0] == '1' && *argv[1] == '1') pidController = &SuspensionGimbalSKD::pitch_v2i_pid;
    else if (*argv[0] == '2' && *argv[1] == '1') pidController = &SuspensionGimbalSKD::BL_v2i_pid;
    else {
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    pidController->change_parameters({Shell::atof(argv[2]),
                                     Shell::atof(argv[3]),
                                     Shell::atof(argv[4]),
                                     Shell::atof(argv[5]),
                                     Shell::atof(argv[6])});
    SuspensionGimbalSKD::yaw_v2i_pid.clear_i_out();
    SuspensionGimbalSKD::yaw_a2v_pid.clear_i_out();
    SuspensionGimbalSKD::pitch_v2i_pid.clear_i_out();
    SuspensionGimbalSKD::pitch_a2v_pid.clear_i_out();
    SuspensionGimbalSKD::BL_v2i_pid.clear_i_out();
    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

/**
 * @brief helper function for cmd_gimbal_echo_parameters()
 * @param chp
 * @param pidController
 */
static inline void _cmd_gimbal_echo_parameters(BaseSequentialStream *chp, PIDControllerBase::pid_params_t pidController) {
    chprintf(chp, "%f %f %f %f %f" SHELL_NEWLINE_STR,
             pidController.kp,
             pidController.ki,
             pidController.kd,
             pidController.i_limit,
             pidController.out_limit);
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
    _cmd_gimbal_echo_parameters(chp, SuspensionGimbalSKD::yaw_a2v_pid.get_parameters());
    chprintf(chp, "yaw v_to_i:       ");
    _cmd_gimbal_echo_parameters(chp, SuspensionGimbalSKD::yaw_v2i_pid.get_parameters());
    chprintf(chp, "pitch angle_to_v: ");
    _cmd_gimbal_echo_parameters(chp, SuspensionGimbalSKD::pitch_a2v_pid.get_parameters());
    chprintf(chp, "pitch v_to_i:     ");
    _cmd_gimbal_echo_parameters(chp, SuspensionGimbalSKD::pitch_v2i_pid.get_parameters());
    chprintf(chp, "bullet_loader v_to_i:     ");
    _cmd_gimbal_echo_parameters(chp, SuspensionGimbalSKD::BL_v2i_pid.get_parameters());
}

static void cmd_gimbal_continuous_shooting(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "g_continuous 0/1");
        return;
    }
    if (*argv[0] - '0'){
        SuspensionGimbalSKD::start_continuous_shooting();
    } else{
        SuspensionGimbalSKD::stop_continuous_shooting();
    }
}

static void cmd_gimbal_incontinuous_shooting(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "g_incontinuous bullet_number");
        return;
    }
    int bullet_num = Shell::atoi(argv[0]);
    if (bullet_num > 0) SuspensionGimbalSKD::start_incontinuous_shooting(bullet_num);
}


static void cmd_fix_front(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1){
        shellUsage(chp, "g_fix 0/1");
        return;
    }
    fix_front = *argv[0] - '0';
    if (fix_front){
        SuspensionGimbalSKD::set_front(SuspensionGimbalIF::YAW_ID);
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
        {"g_continuous",   cmd_gimbal_continuous_shooting},
        {"g_incontinuous", cmd_gimbal_incontinuous_shooting},
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
            if (SuspensionGimbalIF::yaw.enabled || SuspensionGimbalIF::pitch.enabled ||
                    SuspensionGimbalIF::bullet_loader.enabled) {
                // Calculate target signal
                if (enable_angle_to_v_pid){
                    //SuspensionGimbalSKD::set_target_signal();
                } else{
                    if (SuspensionGimbalIF::yaw.enabled) {
                        SuspensionGimbalSKD::set_target_signal(SuspensionGimbalIF::YAW_ID,
                                                                      (int16_t) SuspensionGimbalSKD::yaw_v2i_pid.calc(
                                                                              SuspensionGimbalIF::yaw.angular_velocity,
                                                                              yaw_target_velocity));
                    }
                    if (SuspensionGimbalIF::pitch.enabled) {
                        SuspensionGimbalSKD::set_target_signal(SuspensionGimbalIF::PIT_ID,
                                                                      (int16_t) SuspensionGimbalSKD::pitch_v2i_pid.calc(
                                                                              SuspensionGimbalIF::pitch.angular_velocity,
                                                                              pitch_target_velocity));
                    }
                    if (SuspensionGimbalIF::bullet_loader.enabled) {
                        SuspensionGimbalSKD::set_target_signal(SuspensionGimbalIF::BULLET_LOADER_ID,
                                                                      (int16_t) SuspensionGimbalSKD::BL_v2i_pid.calc(
                                                                              SuspensionGimbalIF::bullet_loader.angular_velocity,
                                                                              BULLET_LOADER_SPEED));
                    }
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

    palSetPad(GPIOH, GPIOH_POWER1_CTRL);
    palSetPad(GPIOH, GPIOH_POWER2_CTRL);
    palSetPad(GPIOH, GPIOH_POWER3_CTRL);
    palSetPad(GPIOH, GPIOH_POWER4_CTRL);
    //MPU6500Controller::start(HIGHPRIO - 3);

    gimbalFeedbackThread.start(NORMALPRIO);

    can1.start(HIGHPRIO - 1);

    SuspensionGimbalIF::init(&can1,&ahrsExt,0,0);

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