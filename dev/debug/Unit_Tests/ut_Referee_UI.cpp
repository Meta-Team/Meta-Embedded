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
#include "referee_UI_logic.h"

// Other headers here

using namespace chibios_rt;

static void cmd_echo_shape(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_shape");
        return;
    }
    RefereeUISKD::echo_shapes();
    chprintf(chp, "echo" SHELL_NEWLINE_STR);
}

static void cmd_add_shape(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "add_shape (name str) (int layer)");
        return;
    }
    char tempchar[3] = {argv[0][0], argv[0][1], argv[0][2]};
    RefereeUISKD::add_rect(tempchar, Shell::atoi(argv[1]), RefereeUISKD::PINK,
                                {860, 380}, {1060, 580}, 10);
    //RefereeUISKD::add_circle(tempchar, Shell::atoi(argv[1]), RefereeUISKD::ACCENT_COLOR, {960,540}, 50,5);
    chprintf(chp, "echo" SHELL_NEWLINE_STR);
}

static void cmd_echo_title(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_title");
        return;
    }
    RefereeUISKD::echo_titles();
    chprintf(chp, "echo" SHELL_NEWLINE_STR);
}

static void cmd_add_title(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3) {
        shellUsage(chp, "add_title (str name) (str string) (int layer)");
        return;
    }
    char compName[3] = {argv[0][0], argv[0][1], argv[0][2]};
    char* string = argv[1];
    RefereeUISKD::add_character(compName, {960, 480}, RefereeUISKD::ACCENT_COLOR, Shell::atoi(argv[2]), 20, 2, string);
    chprintf(chp, "echo" SHELL_NEWLINE_STR);
}

static void cmd_delete_layer(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "remove (int layer)");
        return;
    }
    RefereeUISKD::remove_layer(Shell::atoi(argv[0]));
    chprintf(chp,"removed" SHELL_NEWLINE_STR);
}

static void cmd_revise_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if(argc != 1) {
        shellUsage(chp, "angle (angle)");
        return;
    }
    RefereeUILG::set_chassis_angle(Shell::atof(argv[0]));
    chprintf(chp, "chassis angle set" SHELL_NEWLINE_STR);
}

static void cmd_add_float(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3) {
        shellUsage(chp, "add_float (title) (number) (int layer)");
        return;
    }
    char compName[3] = {argv[0][0], argv[0][1], argv[0][2]};
    float data = Shell::atof(argv[1]);
    RefereeUISKD::add_float(compName, Shell::atoi(argv[2]), RefereeUISKD::GREEN, {100,540}, 20, data);
    chprintf(chp, "echo" SHELL_NEWLINE_STR);
}

static void cmd_add_int(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 3) {
        shellUsage(chp, "add_int (title) (number) (int layer)");
        return;
    }
    char compName[3] = {argv[0][0], argv[0][1], argv[0][2]};
    float data = Shell::atoi(argv[1]);
    RefereeUISKD::add_int(compName, Shell::atoi(argv[2]), RefereeUISKD::GREEN, {100,540}, 20, data);
    chprintf(chp, "echo" SHELL_NEWLINE_STR);
}

// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"echo_shape",      cmd_echo_shape},
        {"add_shape",       cmd_add_shape},
        {"echo_title",      cmd_echo_title},
        {"add_title",       cmd_add_title},
        {"add_float",       cmd_add_float},
        {"add_int",         cmd_add_int},
        {"remove",          cmd_delete_layer},
        {"angle",           cmd_revise_angle},
        {nullptr,   nullptr}
};


// Thread to ...


int main(void) {
    halInit();
    System::init();
    Referee::init(NORMALPRIO);
    RefereeUISKD::init(NORMALPRIO+1);
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);
    chThdSleepMilliseconds(200);
    RefereeUILG::init(NORMALPRIO + 2);


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
