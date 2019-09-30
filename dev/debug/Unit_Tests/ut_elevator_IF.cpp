//
// Created by liuzikai on 2019-01-18.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "can_interface.h"
#include "elevator_interface.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

/**
 * @brief echo actual angular velocity and target current of each motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_elevator_echo(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_echo");
        return;
    }
    chprintf(chp, "accumulate_angle: R = %d, L = %d" SHELL_NEWLINE_STR,
             ElevatorInterface::feedback[ElevatorInterface::R].accumulate_angle,
             ElevatorInterface::feedback[ElevatorInterface::L].accumulate_angle);
    chprintf(chp, "actual_velocity: R = %.2f, L = %.2f" SHELL_NEWLINE_STR,
             ElevatorInterface::feedback[ElevatorInterface::R].actual_velocity,
             ElevatorInterface::feedback[ElevatorInterface::L].actual_velocity);
    chprintf(chp, "target_current: R = %d, L = %d" SHELL_NEWLINE_STR,
             ElevatorInterface::target_current[ElevatorInterface::R],
             ElevatorInterface::target_current[ElevatorInterface::L]);
}

/**
 * @brief set and send target current of each motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_elevator_set_target_currents(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "c_set_current R L");
        return;
    }

    ElevatorInterface::target_current[ElevatorInterface::R] = Shell::atoi(argv[0]);
    ElevatorInterface::target_current[ElevatorInterface::L] = Shell::atoi(argv[1]);
    chprintf(chp, "target_current: R = %d, L = %d" SHELL_NEWLINE_STR,
             ElevatorInterface::target_current[ElevatorInterface::R],
             ElevatorInterface::target_current[ElevatorInterface::L]);

    ElevatorInterface::send_elevator_currents();
    chprintf(chp, "Elevator target_current sent" SHELL_NEWLINE_STR);
}

// Shell commands to control the elevator
ShellCommand elevatorCommands[] = {
        {"e_echo", cmd_elevator_echo},
        {"e_set_current",    cmd_elevator_set_target_currents},
        {nullptr,    nullptr}
};

class GimbalThread : public BaseStaticThread<256> {
protected:
    void main() final {
        setName("elevator");
        while (!shouldTerminate()) {

            ElevatorInterface::send_elevator_currents();

            sleep(TIME_MS2I(100));
        }
    }
} elevatorThread;


int main(void) {
    halInit();
    System::init();
    LED::green_off();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(elevatorCommands);

    can1.start(HIGHPRIO - 1);
    can2.start(HIGHPRIO - 2);
    ElevatorInterface::init(&can2);

    elevatorThread.start(NORMALPRIO);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled,
    // main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow
    // enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}