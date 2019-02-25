//
// Created by liuzikai on 2018/8/6.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "gimbal_interface.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);
int const gimbal_feedback_interval = 25; // ms

class GimbalFeedbackThread : public chibios_rt::BaseStaticThread<512> {

public:

    GimbalFeedbackThread() {
        enable_yaw_feedback = false;
        enable_pitch_feedback = false;
    }

    bool enable_yaw_feedback;
    bool enable_pitch_feedback;

private:

    void main() final {
        setName("gimbal_fb");
        while (!shouldTerminate()) {

            if (enable_yaw_feedback) {
                Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              TIME_I2MS(chibios_rt::System::getTime()),
                              GimbalInterface::yaw.actual_angle, 0.0f,
                              GimbalInterface::yaw.angular_velocity, 0.0f,
                              GimbalInterface::yaw.actual_current, GimbalInterface::yaw.target_current);
//        Shell::printf("yaw round = %d" SHELL_NEWLINE_STR,
//                      GimbalInterface::yaw.round_count);
            }

            if (enable_pitch_feedback) {
                Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              TIME_I2MS(chibios_rt::System::getTime()),
                              GimbalInterface::pitch.actual_angle, 0.0f,
                              GimbalInterface::pitch.angular_velocity, 0.0f,
                              GimbalInterface::pitch.actual_current, GimbalInterface::pitch.target_current);
//        Shell::printf("pitch round = %d" SHELL_NEWLINE_STR,
//                      GimbalInterface::pitch.round_count);
            }

            sleep(TIME_MS2I(gimbal_feedback_interval));
        }

    }

} gimbalFeedbackThread;

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 4 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1')
        || (*argv[2] != '0' && *argv[2] != '1') || (*argv[3] != '0' && *argv[3] != '1')) {
        shellUsage(chp, "g_enable yaw(0/1) pitch(0/1) bullet_loader(0/1) friction(0/1)");
        return;
    }
    GimbalInterface::yaw.enabled = *argv[0] - '0';
    GimbalInterface::pitch.enabled = *argv[1] - '0';
    GimbalInterface::bullet_loader.enabled = *argv[2] - '0';
    GimbalInterface::friction_wheels.enabled = *argv[3] - '0';

    chprintf(chp, "Gimbal yaw enabled = %d" SHELL_NEWLINE_STR, GimbalInterface::yaw.enabled);
    chprintf(chp, "Gimbal pitch enabled = %d" SHELL_NEWLINE_STR, GimbalInterface::pitch.enabled);
    chprintf(chp, "Gimbal bullet_loader enabled = %d" SHELL_NEWLINE_STR, GimbalInterface::bullet_loader.enabled);
    chprintf(chp, "Gimbal friction enabled = %d" SHELL_NEWLINE_STR, GimbalInterface::friction_wheels.enabled);
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

    chprintf(chp, "Gimbal actual angle clear!" SHELL_NEWLINE_STR);
}


/**
 * @brief set target currents of yaw and pitch
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_set_target_currents(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3) {
        shellUsage(chp, "g_set yaw_current pitch_current bullet_loader_current");
        return;
    }

    GimbalInterface::yaw.target_current = Shell::atoi(argv[0]);
    GimbalInterface::pitch.target_current = Shell::atoi(argv[1]);
    GimbalInterface::bullet_loader.target_current = Shell::atoi(argv[2]);
    chprintf(chp, "Gimbal yaw target_current = %d" SHELL_NEWLINE_STR, GimbalInterface::yaw.target_current);
    chprintf(chp, "Gimbal pitch target_current = %d" SHELL_NEWLINE_STR, GimbalInterface::pitch.target_current);
    chprintf(chp, "Gimbal bullet loader target_current = %d" SHELL_NEWLINE_STR,
             GimbalInterface::bullet_loader.target_current);
}

/**
 * @brief set target current of bullet loader
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_set_friction_wheels(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "g_set_fw duty_cycle(0 - 1)");
        return;
    }

    float duty_cycle = Shell::atof(argv[0]);
    if (duty_cycle < 0.0f || duty_cycle > 1.0f) {
        shellUsage(chp, "g_set_fw duty_cycle(0 - 1)");
        return;
    }

    GimbalInterface::friction_wheels.duty_cycle = duty_cycle;
    chprintf(chp, "Gimbal friction_wheels duty_cycle = %f" SHELL_NEWLINE_STR,
             GimbalInterface::friction_wheels.duty_cycle);

    GimbalInterface::send_gimbal_currents();
    chprintf(chp, "Gimbal target_current sent" SHELL_NEWLINE_STR);
}

// Shell commands to pause and resume the echos.
ShellCommand remoteShellCommands[] = {
        {"g_enable", cmd_gimbal_enable},
        {"g_fix",    cmd_gimbal_fix_front_angle},
        {"g_set",    cmd_gimbal_set_target_currents},
        {"g_set_fw", cmd_gimbal_set_friction_wheels},
        {nullptr,    nullptr}
};

class GimbalThread : public BaseStaticThread <256> {
protected:
    void main() final {
        setName("gimbal");
        while (!shouldTerminate()) {
            GimbalInterface::send_gimbal_currents();
            sleep(TIME_MS2I(100));
        }
    }
} gimbalThread;

int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.

    Shell::start(HIGHPRIO);
    Shell::addCommands(remoteShellCommands);

    gimbalThread.start(NORMALPRIO + 1);

    can1.start(HIGHPRIO - 1);
    GimbalInterface::init(&can1);

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
