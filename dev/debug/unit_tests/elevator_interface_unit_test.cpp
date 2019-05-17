//
// Created by liuzikai on 2019-01-18.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
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
    chprintf(chp, "accmulate_angle: FR = %d, FL = %d, BL = %d, BR = %d" SHELL_NEWLINE_STR,
             ElevatorInterface::feedback[ElevatorInterface::FR].accmulate_angle,
             ElevatorInterface::feedback[ElevatorInterface::FL].accmulate_angle,
             ElevatorInterface::feedback[ElevatorInterface::BL].accmulate_angle,
             ElevatorInterface::feedback[ElevatorInterface::BR].accmulate_angle);
    chprintf(chp, "actual_velocity: FR = %.2f, FL = %.2f, BL = %.2f, BR = %.2f" SHELL_NEWLINE_STR,
             ElevatorInterface::feedback[ElevatorInterface::FR].actual_velocity,
             ElevatorInterface::feedback[ElevatorInterface::FL].actual_velocity,
             ElevatorInterface::feedback[ElevatorInterface::BL].actual_velocity,
             ElevatorInterface::feedback[ElevatorInterface::BR].actual_velocity);
    chprintf(chp, "target_current: FR = %d, FL = %d, BL = %d, BR = %d" SHELL_NEWLINE_STR,
             ElevatorInterface::target_current[ElevatorInterface::FR],
             ElevatorInterface::target_current[ElevatorInterface::FL],
             ElevatorInterface::target_current[ElevatorInterface::BL],
             ElevatorInterface::target_current[ElevatorInterface::BR]);
}

/**
 * @brief set and send target current of each motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_elevator_set_target_currents(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 4) {
        shellUsage(chp, "c_set_current FR FL BL BR");
        return;
    }

    ElevatorInterface::target_current[ElevatorInterface::FR] = Shell::atoi(argv[0]);
    ElevatorInterface::target_current[ElevatorInterface::FL] = Shell::atoi(argv[1]);
    ElevatorInterface::target_current[ElevatorInterface::BL] = Shell::atoi(argv[2]);
    ElevatorInterface::target_current[ElevatorInterface::BR] = Shell::atoi(argv[3]);
    chprintf(chp, "target_current: FR = %d, FL = %d, BL = %d, BR = %d" SHELL_NEWLINE_STR,
             ElevatorInterface::target_current[ElevatorInterface::FR],
             ElevatorInterface::target_current[ElevatorInterface::FL],
             ElevatorInterface::target_current[ElevatorInterface::BL],
             ElevatorInterface::target_current[ElevatorInterface::BR]);

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