//
// Created by liuzikai on 2019-07-12.
//

#include "settings_engineer.h"

void cmd_echo_dms(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_dms");
        return;
    }
    chprintf(chp, "FR, FL, BL, BR: %u %u %u %u" SHELL_NEWLINE_STR,
            palReadPad(GPIOC, GPIOC_PIN2),
            palReadPad(GPIOC, GPIOA_PIN3),
            palReadPad(GPIOC, GPIOC_PIN4),
            palReadPad(GPIOC, GPIOC_PIN5));
}

ShellCommand mainProgramCommands[] = {
        {"echo_dms", cmd_echo_dms},
        {"door", cmd_change_door},
        {"echo_sd", cmd_echo_sdcard},
        {"clear_sd", cmd_clear_sdcard},
        {nullptr,    nullptr}
};