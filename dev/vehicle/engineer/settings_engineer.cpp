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
    adcsample_t data[4];
    DMSInterface::get_raw_sample(data);
    chprintf(chp, "FR, FL, BL, BR: %u %u %u %u" SHELL_NEWLINE_STR, data[DMSInterface::FR], data[DMSInterface::FL],
             data[DMSInterface::BL], data[DMSInterface::BR]);
}


ShellCommand mainProgramCommands[] = {
        {"echo_dms", cmd_echo_dms},
        {nullptr,    nullptr}
};