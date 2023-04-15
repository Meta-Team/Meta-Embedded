//
// Created by liuzikai on 2019-02-10.
// Modified by Tony Zhang on 4/15/2023

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"
#include "buzzer_scheduler.h"
#include "chprintf.h"


using namespace chibios_rt;


DEF_SHELL_CMD_START(cmd_play)
    (void) argv;
    if (argc != 1) {
        return false;
    }

    switch (Shell::atoi(argv[0]))
    {
        case 0:
            BuzzerSKD::play_sound(BuzzerSKD::sound_startup);
            break;
        case 1:
            BuzzerSKD::play_sound(BuzzerSKD::sound_startup_intel);
            break;
        case 2:
            BuzzerSKD::play_sound(BuzzerSKD::sound_little_star);
            break;
        case 3:
            BuzzerSKD::play_sound(BuzzerSKD::sound_orange);
            break;
        case 4:
            BuzzerSKD::play_sound(BuzzerSKD::sound_infinity_warning);
            break;
        case 5:
            BuzzerSKD::play_sound(BuzzerSKD::sound_da_bu_zi_duo_ge);
            break;
        case 6:
            BuzzerSKD::play_sound(BuzzerSKD::sound_kong_fu_FC);
            break;
        case 7:
            BuzzerSKD::play_sound(BuzzerSKD::sound_nyan_cat);
            break;
        default:
            chprintf(chp,"buzzer 0-7");
    }
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_alarm)
    (void) argv;
    if (argc != 1) {
        return false;
    }
    switch (Shell::atoi(argv[0]))
    {
        case 0:
            BuzzerSKD::alert_off();
            break;
        case 1:
            BuzzerSKD::alert_on();
            break;
    }
    return true; // command executed successfully
DEF_SHELL_CMD_END

Shell::Command buzzerShellCommands[] = {
        {"buzzer", "buzzer 0-7", cmd_play, nullptr},
        {"alarm" , "alarm (0/1)", cmd_alarm, nullptr},
        {nullptr,    nullptr,nullptr,    nullptr}
};

int main(void) {
    halInit();
    System::init();
    LED::led_on(1);
    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.

    Shell::start(HIGHPRIO);
    Shell::addCommands(buzzerShellCommands);

    BuzzerSKD::init(NORMALPRIO);

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
