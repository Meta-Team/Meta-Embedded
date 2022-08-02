//
// Created by liuzikai on 2019-07-12.
//

#ifndef META_INFANTRY_SETTINGS_INFANTRY_H
#define META_INFANTRY_SETTINGS_INFANTRY_H

#include "ch.hpp"
#include "hal.h"

#include "shell.h"

#include "user_engineer.h"
#include "engineer_interface.h"

extern ShellCommand mainProgramCommands[];

void cmd_echo_dms(BaseSequentialStream *chp, int argc, char *argv[]);

void cmd_change_door(BaseSequentialStream *chp, int argc, char *argv[]);

void cmd_echo_sdcard(BaseSequentialStream *chp, int argc, char *argv[]);

void cmd_clear_sdcard(BaseSequentialStream *chp, int argc, char *argv[]);



#endif //META_INFANTRY_SETTINGS_INFANTRY_H
