//
// Created by 钱晨 on 2020/1/19.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "can_interface.h"
// Other headers here

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_set_motor_type(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3) {
        shellUsage(chp, "set  (1/2)(can channel) (id)(motor id) (num)(1-4)");
        return;
    }
    if (Shell::atoi(argv[0]) == 1) {
        can1.set_motor_type(Shell::atoi(argv[1]), (CANInterface::motor_type_t) Shell::atoi(argv[2]));
    } else if (Shell::atoi(argv[0]) == 2) {
        can2.set_motor_type(Shell::atoi(argv[1]), (CANInterface::motor_type_t) Shell::atoi(argv[2]));
    }
    chprintf(chp, "set!" SHELL_NEWLINE_STR);
}

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_echo_adress(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if(argc != 1) {
        shellUsage(chp, "echo_adress id(motor id)");
        return;
    }
    Shell::printf("%d" SHELL_NEWLINE_STR, can1.get_feedback_address(1));
    Shell::printf("%d" SHELL_NEWLINE_STR, can2.get_feedback_address(1));
    chprintf(chp, "echo!" SHELL_NEWLINE_STR);
}

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_echo_motor_ID(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "set (1/2)(can channel) (id)(motor id)");
        return;
    }
    if (Shell::atoi(argv[0]) == 1) {
        Shell::printf("%d" SHELL_NEWLINE_STR, can1.get_feedback_address(Shell::atoi(argv[1]))->type);
    } else if (Shell::atoi(argv[0]) == 2) {
        Shell::printf("%d" SHELL_NEWLINE_STR, can2.get_feedback_address(Shell::atoi(argv[1]))->type);
    }
    chprintf(chp, "echoed!" SHELL_NEWLINE_STR);
}

static void cmd_set_current(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3) {
        shellUsage(chp, "set  (1/2)(can channel) (id)(motor id) (current)(1-4)");
        return;
    }
    if (Shell::atoi(argv[0]) == 1) {
        can1.set_target_current(Shell::atoi(argv[1]), (CANInterface::motor_type_t) Shell::atoi(argv[2]));
    } else if (Shell::atoi(argv[0]) == 2) {
        can2.set_target_current(Shell::atoi(argv[1]), (CANInterface::motor_type_t) Shell::atoi(argv[2]));
    }
    chprintf(chp, "echoed!" SHELL_NEWLINE_STR);
}

static void cmd_echo_current(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "set (1/2)(can channel) (id)(motor id)");
        return;
    }
    if (Shell::atoi(argv[0]) == 1) {
        Shell::printf("%d" SHELL_NEWLINE_STR, can1.echo_target_current(Shell::atoi(argv[1])));
    } else if (Shell::atoi(argv[0]) == 2) {
        Shell::printf("%d" SHELL_NEWLINE_STR, can2.echo_target_current(Shell::atoi(argv[1])));
    }
    chprintf(chp, "echoed!" SHELL_NEWLINE_STR);
}

// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"set", cmd_set_motor_type},
        {"echo_adress", cmd_echo_adress},
        {"type", cmd_echo_motor_ID},
        {"current", cmd_set_current},
        {"echo_current", cmd_echo_current},
        {nullptr,    nullptr}
};

// Thread to ...
class CANInterfaceDebugThread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("ut_CAN");
        while (!shouldTerminate()) {

            sleep(TIME_MS2I(100));
        }
    }
} CANInterfaceDebugThread;


int main(void) {
    halInit();
    System::init();

    can1.start(NORMALPRIO+2, NORMALPRIO+3);
    can2.start(NORMALPRIO+4, NORMALPRIO+5);
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);

    CANInterfaceDebugThread.start(NORMALPRIO + 1);


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

