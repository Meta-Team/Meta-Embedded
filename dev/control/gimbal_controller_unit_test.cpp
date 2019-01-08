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
#include "gimbal_feedback_module.h"

using namespace chibios_rt;

/**
 * @brief callback function for CAN1
 * @param rxmsg
 */
static void can1_callback(CANRxFrame *rxmsg) {
    switch (rxmsg->SID) {
        case 0x205:
        case 0x206:
            GimbalInterface::process_motor_feedback(rxmsg);
            break;
        default:
            break;
    }
}

// Calculation interval for gimbal thread
int const gimbal_thread_interval = 50; // ms
int const maximum_current = 4000; // mA

float const yaw_min_angle = -120; // degree
float const yaw_max_angle = 120; // degree
float const pitch_min_angle = -45; // degree
float const pitch_max_angle = 45; // degree

float const yaw_max_speed = 200; // absolute maximum, degree/s
float const pitch_max_speed = 100; // absolute maximum, degree/s

bool enable_angle_to_v_pid = false;

float yaw_target_angle = 0.0;
float yaw_target_velocity = 0.0;
float pitch_target_angle = 0.0;
float pitch_target_velocity = 0.0;

CANInterface can1(&CAND1, can1_callback);
GimbalFeedbackModule feedbackModule(200,  // 200ms interval
                                    &yaw_target_angle,
                                    &yaw_target_velocity,
                                    &GimbalInterface::yaw.target_current,
                                    &pitch_target_angle,
                                    &pitch_target_velocity,
                                    &GimbalInterface::pitch.target_current);


/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1')) {
        shellUsage(chp, "g_enable yaw(0/1) pitch(0/1)");
        return;
    }
    GimbalInterface::yaw.enabled = *argv[0] - '0';
    GimbalInterface::pitch.enabled = *argv[1] - '0';

    chprintf(chp, "Gimbal yaw enabled = %d" SHELL_NEWLINE_STR, GimbalInterface::yaw.enabled);
    chprintf(chp, "Gimbal pitch enabled = %d" SHELL_NEWLINE_STR, GimbalInterface::pitch.enabled);
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

    chprintf(chp, "Gimbal actual angle clear!" SHELL_NEWLINE_STR);
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
    chprintf(chp, "Gimbal yaw target_velocity = %f" SHELL_NEWLINE_STR, yaw_target_velocity);
    chprintf(chp, "Gimbal pitch target_velocity = %f" SHELL_NEWLINE_STR, pitch_target_velocity);

    enable_angle_to_v_pid = false;
    chprintf(chp, "Gimbal pos_to_v_pid disabled" SHELL_NEWLINE_STR);
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
    chprintf(chp, "Gimbal yaw target_angle = %f" SHELL_NEWLINE_STR, yaw_target_angle);
    chprintf(chp, "Gimbal pitch target_angle = %f" SHELL_NEWLINE_STR, pitch_target_angle);

    enable_angle_to_v_pid = true;
    chprintf(chp, "Gimbal pos_to_v_pid enabled" SHELL_NEWLINE_STR);
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
        shellUsage(chp, "g_set_params yaw(0)/pitch(1) angle_to_v(0)/v_to_i(0) ki kp kd i_limit out_limit");
        return;
    }

    PIDController *pidController = nullptr;
    if (*argv[0] == '0' && *argv[1] == '0') pidController = &GimbalController::yaw.angle_to_v_pid;
    else if (*argv[0] == '0' && *argv[1] == '1') pidController = &GimbalController::yaw.v_to_i_pid;
    else if (*argv[0] == '1' && *argv[1] == '0') pidController = &GimbalController::pitch.angle_to_v_pid;
    else if (*argv[0] == '1' && *argv[1] == '1') pidController = &GimbalController::pitch.v_to_i_pid;
    else {
        chprintf(chp, "Error arguments!" SHELL_NEWLINE_STR);
        return;
    }

    pidController->change_parameters(Shell::atof(argv[2]),
                                     Shell::atof(argv[3]),
                                     Shell::atof(argv[4]),
                                     Shell::atof(argv[5]),
                                     Shell::atof(argv[6]));
    chprintf(chp, "Parameters set" SHELL_NEWLINE_STR);
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
}

// Command lists for gimbal controller test and adjustments
ShellCommand gimbalCotrollerCommands[] = {
        {"g_enable",      cmd_gimbal_enable},
        {"g_fix",         cmd_gimbal_fix_front_angle},
        {"g_set_v",       cmd_gimbal_set_target_velocities},
        {"g_set_angle",   cmd_gimbal_set_target_angle},
        {"g_set_params",  cmd_gimbal_set_parameters},
        {"g_echo_params", cmd_gimbal_echo_parameters},
        {nullptr,         nullptr}
};

/**
 * Main calculation thread
 */
class GimbalThread : public BaseStaticThread<256> {
protected:
    void main() final {
        setName("gimbal");
        while (!shouldTerminate()) {
            
            if (GimbalInterface::yaw.enabled || GimbalInterface::pitch.enabled)
            {
                
                // Perform angle check
                if (GimbalInterface::yaw.actual_angle > yaw_max_angle) {
                    Shell::printf("[WARNING] Yaw reach max angle. Disabled." SHELL_NEWLINE_STR);
                    GimbalInterface::yaw.enabled = false;
                    GimbalInterface::send_gimbal_currents();
                    continue; // make sure there is no chSysLock() before
                } else if (GimbalInterface::yaw.actual_angle < yaw_min_angle) {
                    Shell::printf("[WARNING] Yaw reach min angle. Disabled." SHELL_NEWLINE_STR);
                    GimbalInterface::yaw.enabled = false;
                    GimbalInterface::send_gimbal_currents();
                    continue; // make sure there is no chSysLock() before
                }
                if (GimbalInterface::pitch.actual_angle > pitch_max_angle) {
                    Shell::printf("[WARNING] Pitch reach max angle. Disabled." SHELL_NEWLINE_STR);
                    GimbalInterface::pitch.enabled = false;
                    GimbalInterface::send_gimbal_currents();
                    continue; // make sure there is no chSysLock() before
                } else if (GimbalInterface::pitch.actual_angle < pitch_min_angle) {
                    Shell::printf("[WARNING] Pitch reach min angle. Disabled." SHELL_NEWLINE_STR);
                    GimbalInterface::pitch.enabled = false;
                    GimbalInterface::send_gimbal_currents();
                    continue; // make sure there is no chSysLock() before
                }

                
                // Calculate target velocity
                float _yaw_target_velocity;
                float _pitch_target_velocity;
                if (enable_angle_to_v_pid) {
                    _yaw_target_velocity = GimbalController::yaw.angle_to_v(GimbalInterface::yaw.actual_angle,
                                                                            yaw_target_angle);
                    _pitch_target_velocity = GimbalController::pitch.angle_to_v(GimbalInterface::pitch.actual_angle,
                                                                                pitch_target_angle);
                } else {
                    _yaw_target_velocity = yaw_target_velocity;
                    _pitch_target_velocity = pitch_target_velocity;
                }

                // Perform velocity check
                if (GimbalInterface::yaw.angular_velocity > yaw_max_speed || GimbalInterface::yaw.angular_velocity < -yaw_max_speed) {
                    Shell::printf("[WARNING] Yaw reach max velocity. Disabled." SHELL_NEWLINE_STR);
                    GimbalInterface::yaw.enabled = false;
                    GimbalInterface::send_gimbal_currents();
                    continue; // make sure there is no chSysLock() before
                }
                if (GimbalInterface::pitch.angular_velocity > pitch_max_speed || GimbalInterface::pitch.angular_velocity < -pitch_max_speed) {
                    Shell::printf("[WARNING] Pitch reach max velocity. Disabled." SHELL_NEWLINE_STR);
                    GimbalInterface::pitch.enabled = false;
                    GimbalInterface::send_gimbal_currents();
                    continue; // make sure there is no chSysLock() before
                }

                // Calculate target current
                GimbalInterface::yaw.target_current = (int) GimbalController::yaw.v_to_i(
                        GimbalInterface::yaw.angular_velocity, _yaw_target_velocity);
                GimbalInterface::pitch.target_current = (int) GimbalController::pitch.v_to_i(
                        GimbalInterface::pitch.angular_velocity, _pitch_target_velocity);

                // Perform current check
                if (GimbalInterface::yaw.target_current > maximum_current || GimbalInterface::yaw.target_current < -maximum_current) {
                    Shell::printf("[WARNING] Yaw reach max current. Disabled." SHELL_NEWLINE_STR);
                    GimbalInterface::yaw.enabled = false;
                    GimbalInterface::send_gimbal_currents();
                    continue; // make sure there is no chSysLock() before
                }
                if (GimbalInterface::pitch.target_current > maximum_current || GimbalInterface::pitch.target_current < -maximum_current) {
                    Shell::printf("[WARNING] Pitch reach max current. Disabled." SHELL_NEWLINE_STR);
                    GimbalInterface::pitch.enabled = false;
                    GimbalInterface::send_gimbal_currents();
                    continue; // make sure there is no chSysLock() before
                }
                
                // Send current

            }

            GimbalInterface::send_gimbal_currents();
            
            sleep(TIME_MS2I(gimbal_thread_interval));
        }
    }
} gimbalThread;

int main(void) {
    halInit();
    System::init();

    Shell::start(HIGHPRIO);
    Shell::addCommands(gimbalCotrollerCommands);

    feedbackModule.start_thread(NORMALPRIO);

    can1.start_can();
    can1.start_thread(HIGHPRIO - 1);
    GimbalInterface::set_can_interface(&can1);

    gimbalThread.start(HIGHPRIO - 2);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled,
    // main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow
        // enter an infinite loop, so we set the priority to lowest
        // before quitting, to let other threads run normally
        BaseThread::setPriority(1);
#endif
    return 0;
}

