//
// Created by liuzikai on 2019-02-10.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "buzzer.h"

using namespace chibios_rt;


static void cmd_play(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "buzzer 0-4");
        return;
    }

    switch (Shell::atoi(argv[0]))
    {
        case 0:
            Buzzer::play_sound(Buzzer::sound_startup, NORMALPRIO);
            break;
        case 1:
            Buzzer::play_sound(Buzzer::sound_startup_intel, NORMALPRIO);
            break;
        case 2:
            Buzzer::play_sound(Buzzer::sound_little_star, NORMALPRIO);
            break;
        case 3:
            Buzzer::play_sound(Buzzer::sound_orange, NORMALPRIO);
            break;
        case 4:
            Buzzer::play_sound(Buzzer::sound_infinity_warning, NORMALPRIO);
            break;
        default:
            shellUsage(chp, "buzzer 0-4");
    }

}

ShellCommand buzzerShellCommands[] = {
        {"buzzer", cmd_play},
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
