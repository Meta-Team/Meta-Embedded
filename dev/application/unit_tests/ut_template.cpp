//
// Created by ... on YYYY/MM/DD.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "interface/led/led.h"
#include "debug/shell/shell.h"
// Other headers here

using namespace chibios_rt;

/**
 * @brief Shell command template
 * @param argc Number of input parameters
 * @param argv Input parameters in string.
 */
DEF_SHELL_CMD_START(cmd_template)
    Shell::printf("_t:Template" ENDL);
DEF_SHELL_CMD_END


// Shell commands to ...
Shell::Command templateShellCommands[] = {
        {"_template", "_template [data] [input] [formate]", cmd_template, nullptr},
        {nullptr,    nullptr, nullptr, nullptr} // Null command in the end.
};


// Thread to ...
class TemplateThread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("template");
        while (!shouldTerminate()) {
            sleep(TIME_MS2I(100));
        }
    }
} template_thread;


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);

    template_thread.start(NORMALPRIO + 1);


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
