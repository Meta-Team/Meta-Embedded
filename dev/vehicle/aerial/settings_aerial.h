//
// Created by Kerui Zhu on 7/19/2019.
//

#ifndef META_INFANTRY_SETTINGS_AERIAL_H
#define META_INFANTRY_SETTINGS_AERIAL_H

#include "ch.hpp"
#include "hal.h"

#include "shell.h"

#include "user_aerial.h"

void gimbal_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]);

void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]);

extern ShellCommand mainProgramCommands[];

#endif //META_INFANTRY_SETTINGS_AERIAL_H
