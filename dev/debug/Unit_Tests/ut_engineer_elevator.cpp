//
// Created by 钱晨 on 7/29/21.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "engineer_elevator_skd.h"
#include "engineer_elevator_interface.h"
// Other headers here

using namespace chibios_rt;

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_template(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "set (h/v) dist");
        return;
    }
    if(*argv[0] == 'h') {
        EngineerElevatorSKD::set_target_movement(Shell::atof(argv[1]));
    } else {
        EngineerElevatorSKD::set_target_height(Shell::atof(argv[1]));
    }
    chprintf(chp, "template" SHELL_NEWLINE_STR);
}


// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"set", cmd_template},
        {nullptr,    nullptr}
};


// Thread to ...
class SkywalkerAdjustThread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("template");
        while (!shouldTerminate()) {

            sleep(TIME_MS2I(100));
        }
    }
} templateThread;


int main(void) {
    halInit();
    System::init();

    EngineerElevatorSKD::start(NORMALPRIO, 1.0);
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);

    templateThread.start(NORMALPRIO + 1);


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
