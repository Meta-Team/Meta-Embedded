//
// Created by liuzikai on 2019-02-24.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "robotic_arm.h"

#include "robotic_arm_thread.h"

#include "buzzer.h"

CANInterface can1(&CAND1);
RoboticArmThread roboticArmThread;

static void cmd_robotic_arm_action(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "engi_fetch");
        return;
    }
    roboticArmThread.start_actions(NORMALPRIO);
    chprintf(chp, "Start up action." SHELL_NEWLINE_STR);
}

static void cmd_robotic_arm_emergency_stop(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s (emergency stop)");
        return;
    }
    roboticArmThread.emergency_stop();
    chprintf(chp, "EMERGENCY STOP" SHELL_NEWLINE_STR);
}

// Shell commands to control the chassis
ShellCommand elevatorInterfaceCommands[] = {
        {"engi_fetch", cmd_robotic_arm_action},
        {"s", cmd_robotic_arm_emergency_stop},
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

    chThdSleepMilliseconds(1000);
    RoboticArm::reset_front_angle();

    Buzzer::play_sound(Buzzer::sound_startup, LOWPRIO);

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