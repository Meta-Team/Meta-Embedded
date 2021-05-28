//
// Created by Qian Chen on 5/28/21.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "referee_UI_update_scheduler.h"

// Other headers here

using namespace chibios_rt;

static void cmd_echo_shape(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_shape");
        return;
    }
    referee_UI_update_scheduler::echo_shapes();
    chprintf(chp, "echo" SHELL_NEWLINE_STR);
}

static void cmd_add_shape(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "add_shape (str)");
        return;
    }
    char tempchar[3] = {argv[0][0], argv[0][1], argv[0][2]};
    referee_UI_update_scheduler::add_rect(tempchar, 0,referee_UI_update_scheduler::BLACK, {0,0},{0,0}, 0);
    chprintf(chp, "echo" SHELL_NEWLINE_STR);
}

static void cmd_echo_title(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_title");
        return;
    }
    referee_UI_update_scheduler::echo_titles();
    chprintf(chp, "echo" SHELL_NEWLINE_STR);
}

static void cmd_add_title(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "add_title (str) (str)");
        return;
    }
    char compName[3] = {argv[0][0], argv[0][1], argv[0][2]};
    char* string = argv[1];
    referee_UI_update_scheduler::add_character(compName, {0,0}, referee_UI_update_scheduler::BLACK, 0,0,0, string);
    chprintf(chp, "echo" SHELL_NEWLINE_STR);
}

// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"echo_shape",      cmd_echo_shape},
        {"add_shape",       cmd_add_shape},
        {"echo_title",      cmd_echo_title},
        {"add_title",       cmd_add_title} ,
        {nullptr,   nullptr}
};


// Thread to ...


int main(void) {
    halInit();
    System::init();
    Referee::init(NORMALPRIO);
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);


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
