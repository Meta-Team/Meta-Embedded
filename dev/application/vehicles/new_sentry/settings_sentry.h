//
// Created by liuzikai on 2019-07-12.
//

#ifndef META_SENTRY_SETTINGS_H
#define META_SENTRY_SETTINGS_H

#include "ch.hpp"
#include "hal.h"

#include "shell.h"

#include "user_sentry.h"

void gimbal_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]);

void chassis_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
void chassis_set_config(BaseSequentialStream *chp, int argc, char *argv[]);

void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]);

extern ShellCommand mainProgramCommands[];

#endif //META_SENTRY_SETTINGS_H
