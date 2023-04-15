//
// Created by ... on YYYY/MM/DD.
// Modified by Tony Zhang on 4/15/2023

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "interface/led/led.h"
#include "shell.h"
#include "hardware_conf.h"
#include "oled_interface.h"
// Other headers here

using namespace chibios_rt;

// Thread to ...
class OLEDLogoThread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("oled_logo");
        while (!shouldTerminate()) {
            oledIF::oled_clear(oledIF::Pen_Clear);
            oledIF::oled_LOGO();
            oledIF::oled_refresh_gram();
            sleep(TIME_MS2I(500));
        }
    }
} oled_logo_thd;

DEF_SHELL_CMD_START(cmd_start)
    Shell::printf("Cyka Blyat!" SHELL_NEWLINE_STR);
    return true;
DEF_SHELL_CMD_END


// Shell commands to ...
Shell::Command ShellCommands[] = {
        {"_s",              nullptr,    cmd_start,           nullptr},
        {nullptr,           nullptr,    nullptr,                     nullptr}
};


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(ShellCommands);

    oledIF::init();
    oled_logo_thd.start(NORMALPRIO + 1);

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
