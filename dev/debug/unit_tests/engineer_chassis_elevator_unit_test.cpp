//
// Created by liuzikai on 2019-02-24.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "elevator_interface.h"

#include "elevator_thread.h"

CANInterface can1(&CAND1);
ElevatorThread elevatorThread;

static void cmd_elevator_up(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "engi_up");
        return;
    }
    elevatorThread.start_up_actions(NORMALPRIO);
    chprintf(chp, "Start up action." SHELL_NEWLINE_STR);
}

static void cmd_elevator_emergency_stop(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s (emergency stop)");
        return;
    }
    elevatorThread.emergency_stop();
    chprintf(chp, "EMERGENCY STOP" SHELL_NEWLINE_STR);
}

// Shell commands to control the chassis
ShellCommand elevatorInterfaceCommands[] = {
        {"engi_up", cmd_elevator_up},
        {"s", cmd_elevator_emergency_stop},
        {nullptr, nullptr}
};

int main(void) {
    halInit();
    chibios_rt::System::init();

    Shell::start(HIGHPRIO);
    Shell::addCommands(elevatorInterfaceCommands);

    LED::red_off();
    LED::green_off();

    can1.start(HIGHPRIO - 1);

#if CH_CFG_NO_IDLE_THREAD // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled,  main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(1);
#endif
    return 0;
}