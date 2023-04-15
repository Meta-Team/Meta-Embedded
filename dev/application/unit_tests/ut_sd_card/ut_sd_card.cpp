//
// Created by liuzikai on 2019/07/13.
// Modified by Tony Zhang on 4/15/2023
//

/**
 * This file contain SD Card Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "interface/led/led.h"
#include "shell.h"
#include "chprintf.h"

#include "interface/sd_card/sd_card_interface.h"

using namespace chibios_rt;

static const uint16_t TEST_DATA_CMD_ID = 0xEEEE;
__PACKED_STRUCT test_struct_t {
    float x;
    float y;
    float z;
    bool a;
    bool b;
} test_data;

DEF_SHELL_CMD_START(cmd_sd_write)
    (void) argv;
    if (argc != 5) {
        return false;
    }
    test_data.x = Shell::atof(argv[0]);
    test_data.y = Shell::atof(argv[1]);
    test_data.z = Shell::atof(argv[2]);
    test_data.a = Shell::atoi(argv[3]);
    test_data.b = Shell::atoi(argv[4]);
    chprintf(chp, "%d" SHELL_NEWLINE_STR, SDCard::write_data(TEST_DATA_CMD_ID, &test_data, sizeof(test_data)));
    return true;// command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_sd_get)
    (void) argv;
    if (argc != 0) {
        return false;
    }
    chprintf(chp, "%d" SHELL_NEWLINE_STR, SDCard::get_data(TEST_DATA_CMD_ID, &test_data, sizeof(test_data)));
    chprintf(chp, "test_data.x = %f" SHELL_NEWLINE_STR, test_data.x);
    chprintf(chp, "test_data.y = %f" SHELL_NEWLINE_STR, test_data.y);
    chprintf(chp, "test_data.z = %f" SHELL_NEWLINE_STR, test_data.z);
    chprintf(chp, "test_data.a = %d" SHELL_NEWLINE_STR, test_data.a);
    chprintf(chp, "test_data.b = %d" SHELL_NEWLINE_STR, test_data.b);
    return true;// command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_sd_read_all)
    (void) argv;
    if (argc != 0) {
        return false;
    }
    chprintf(chp, "%d" SHELL_NEWLINE_STR, SDCard::read_all());
    return true;// command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_sd_erase)
    (void) argv;
    if (argc != 0) {
        return false;
    }
    chprintf(chp, "%d" SHELL_NEWLINE_STR, SDCard::erase());
    return true; // command executed successfully
DEF_SHELL_CMD_END


Shell::Command sdCommands[] = {
        {"sd_write", "sd_write x y z a b", cmd_sd_write, nullptr},
        {"sd_get","sd_get", cmd_sd_get, nullptr},
        {"sd_read_all", "sd_read_all",cmd_sd_read_all, nullptr},
        {"sd_erase", "sd_erase",cmd_sd_erase, nullptr},
        {nullptr,    nullptr, nullptr, nullptr}
};


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(sdCommands);

    LOG("SDCard::init() = %d", SDCard::init());


#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}
