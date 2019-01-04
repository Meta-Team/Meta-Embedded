//
// Created by liuzikai on 2018/8/6.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "gimbal_interface.h"
#include "gimbal_feedback_module.h"

using namespace chibios_rt;

/**
 * @brief callback function for CAN1
 * @param rxmsg
 */
static void can1_callback (CANRxFrame *rxmsg) {
    switch (rxmsg->SID) {
        case 0x205:
        case 0x206:
            GimbalInterface::process_motor_feedback(rxmsg);
            break;
        default:
            break;
    }
}

float empty_target_angle = 0.0;
float empty_target_velocity = 0.0;

CANInterface can1(&CAND1, can1_callback);
GimbalFeedbackModule feedbackModule (200,  // 200ms interval
        &empty_target_angle,
        &empty_target_velocity,
        &GimbalInterface::yaw.target_current,
        &empty_target_angle,
        &empty_target_velocity,
        &GimbalInterface::pitch.target_current);


/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
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
    (void)argv;
    if (argc != 0) {
        shellUsage(chp, "g_fix");
        return;
    }
    GimbalInterface::yaw.actual_angle = 0;
    GimbalInterface::yaw.round_count = 0;
    GimbalInterface::pitch.actual_angle = 0;
    GimbalInterface::pitch.round_count = 0;

    chprintf(chp, "Gimbal actual angle clear!" SHELL_NEWLINE_STR);
}



/**
 * @brief set target currents of yaw and pitch
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_set_target_currents(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    if (argc != 2) {
        shellUsage(chp, "g_set yaw_current pitch_current");
        return;
    }

    GimbalInterface::yaw.target_current = Shell::atoi(argv[0]);
    GimbalInterface::pitch.target_current = Shell::atoi(argv[1]);
    chprintf(chp, "Gimbal yaw target_current = %d" SHELL_NEWLINE_STR, GimbalInterface::yaw.target_current);
    chprintf(chp, "Gimbal pitch target_current = %d" SHELL_NEWLINE_STR, GimbalInterface::pitch.target_current);

    GimbalInterface::send_gimbal_currents();
    chprintf(chp, "Gimbal target_current sent" SHELL_NEWLINE_STR);
}

// Shell commands to pause and resume the echos.
ShellCommand remoteShellCommands[] = {
        {"g_enable", cmd_gimbal_enable},
        {"g_fix", cmd_gimbal_fix_front_angle},
        {"g_set", cmd_gimbal_set_target_currents},
        {nullptr, nullptr}
};

int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.


    init_led();

    Shell::start(HIGHPRIO);
    Shell::addCommands(remoteShellCommands);

    feedbackModule.start_thread(NORMALPRIO);

    can1.start_can();
    can1.start_thread(HIGHPRIO - 1);
    GimbalInterface::set_can_interface(&can1);

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
