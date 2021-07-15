//
// Created by liuzikai on 2019-07-12.
//

#ifndef META_INFANTRY_SETTINGS_HERO_H
#define META_INFANTRY_SETTINGS_HERO_H

#include "ch.hpp"
#include "hal.h"

#include "shell.h"

#include "user_hero.h"

//void gimbal_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
//void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]);
//
//void chassis_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
//void chassis_set_config(BaseSequentialStream *chp, int argc, char *argv[]);
//
//void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
//void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]);

void cmd_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_disable_feedback(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_set_param(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_echo_param(BaseSequentialStream *chp, int argc, char *argv[]);

void feedback_thread_start();

extern ShellCommand mainProgramCommands[];

#endif //META_INFANTRY_SETTINGS_HERO_H
