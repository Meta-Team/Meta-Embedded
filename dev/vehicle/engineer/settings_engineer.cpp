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

void cmd_change_door(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "door");
        return;
    }
    EngineerInterface::change_door();
}

void cmd_robotic_prev(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "prev");
        return;
    }
    RoboticArmSKD::prev_step();
}

void cmd_echo_sdcard(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_sd");
        return;
    }
    float present_angle;
    chprintf(chp, "%d , present angle: %d, present height: %d" SHELL_NEWLINE_STR, SDCard::get_data(ELEVATOR_ANGLE_DATA_ID, &present_angle, sizeof(present_angle)), present_angle, present_angle/ANGLE_HEIGHT_RATIO);
}

void cmd_clear_sdcard(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "clear_sd");
        return;
    }
    chprintf(chp, "%d" SHELL_NEWLINE_STR, SDCard::erase());
}

void cmd_next_step(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "clear_sd");
        return;
    }
    RoboticArmSKD::next_step();
}

ShellCommand mainProgramCommands[] = {
        {"echo_dms", cmd_echo_dms},
        {"next", cmd_next_step},
        {"prev", cmd_robotic_prev},
        {"door", cmd_change_door},
        {"echo_sd", cmd_echo_sdcard},
        {"clear_sd", cmd_clear_sdcard},
        {nullptr,    nullptr}
};