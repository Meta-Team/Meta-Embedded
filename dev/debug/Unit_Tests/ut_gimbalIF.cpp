//
// Created by ... on YYYY/MM/DD.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "can_interface.h"
#include "scheduler/buzzer_scheduler.h"
#include "interface/gimbal_interface.h"
// Other headers here

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

// Thread to ...
class GimbalThread : public BaseStaticThread <512> {
public:
    bool enable_motor[2] = {false, false};
private:
    void main() final {
        setName("template");
        while (!shouldTerminate()) {

            if(enable_motor[0]) {
                *GimbalIF::target_current[GimbalIF::YAW] = 1000;
            } else {
                *GimbalIF::target_current[GimbalIF::YAW] = 0;
            }
            if(enable_motor[1]) {
                *GimbalIF::target_current[GimbalIF::PITCH] = 1000;
            } else {
                *GimbalIF::target_current[GimbalIF::PITCH] = 0;
            }

            GimbalIF::enable_gimbal_current_clip();
            sleep(TIME_MS2I(1));
        }
    }
} ut_gimbal_thread;

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_print(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "print");
        return;
    }

    Shell::printf("YAW  : angle:%f velocity:%f accumulate angle:%d actual current:%d" SHELL_NEWLINE_STR,
                  GimbalIF::feedback[GimbalIF::YAW]->actual_angle,        GimbalIF::feedback[GimbalIF::YAW]->actual_velocity,
                  GimbalIF::feedback[GimbalIF::YAW]->accumulated_angle(), GimbalIF::feedback[GimbalIF::YAW]->actual_current);
    Shell::printf("PITCH: angle:%f velocity:%f accumulate angle:%d actual current:%d" SHELL_NEWLINE_STR,
                  GimbalIF::feedback[GimbalIF::PITCH]->actual_angle,        GimbalIF::feedback[GimbalIF::PITCH]->actual_velocity,
                  GimbalIF::feedback[GimbalIF::PITCH]->accumulated_angle(), GimbalIF::feedback[GimbalIF::PITCH]->actual_current);

    chprintf(chp, "print" SHELL_NEWLINE_STR);
}

static void cmd_enable_current(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "enable (0/1) yaw/pitch");
        return;
    }
    ut_gimbal_thread.enable_motor[Shell::atoi(argv[0])] = true;
    chprintf(chp, "enable" SHELL_NEWLINE_STR);
}

static void cmd_disable_current(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "disable (0/1) yaw/pitch");
        return;
    }
    ut_gimbal_thread.enable_motor[Shell::atoi(argv[0])] = false;
    chprintf(chp, "print" SHELL_NEWLINE_STR);
}

// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"print", cmd_print},
        {"enable", cmd_enable_current},
        {"disable", cmd_disable_current},
        {nullptr,    nullptr}
};


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);
    BuzzerSKD::init(LOWPRIO);
    can1.start(NORMALPRIO+1, NORMALPRIO+2);
    can2.start(NORMALPRIO+3, NORMALPRIO+4);
    chThdSleepMilliseconds(5);
    GimbalIF::motor_can_config_t canConfig[6] = {{GimbalIF::can_channel_2,4,CANInterface::GM6020},
                                                 {GimbalIF::can_channel_1,1,CANInterface::GM6020},
                                                 {GimbalIF::none_can_channel,6,CANInterface::NONE_MOTOR},
                                                 {GimbalIF::none_can_channel,8,CANInterface::NONE_MOTOR},
                                                 {GimbalIF::none_can_channel,9,CANInterface::NONE_MOTOR},
                                                 {GimbalIF::none_can_channel,10,CANInterface::NONE_MOTOR}};
    GimbalIF::init(&can1, &can2,
                   canConfig,
                   0, 0);
//    BuzzerSKD::play_sound(BuzzerSKD::sound_nyan_cat);
    ut_gimbal_thread.start(NORMALPRIO);


#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}
