//
// Created by liuzikai on 2019-02-10.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "buzzer.h"

using namespace chibios_rt;


static void cmd_play_startup(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "buzzer_startup");
        return;
    }

    Buzzer::play_sound(Buzzer::sound_startup, 250, NORMALPRIO);
}

static void cmd_play_little_star(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "buzzer_little_star");
        return;
    }

    Buzzer::play_sound(Buzzer::sound_little_star, 150, NORMALPRIO);
}

ShellCommand buzzerShellCommands[] = {
        {"buzzer_startup", cmd_play_startup},
        {"buzzer_little_star",    cmd_play_little_star},
        {nullptr,    nullptr}
};

int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.

    Shell::start(HIGHPRIO);
    Shell::addCommands(buzzerShellCommands);

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
