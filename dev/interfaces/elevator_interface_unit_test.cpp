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

/**
 * @brief callback function for CAN1
 * @param rxmsg
 */
static void can1_callback(CANRxFrame *rxmsg) {
    switch (rxmsg->SID) {
        case 0x31B:
            ElevatorInterface::get_feedback(rxmsg);
            LED::green_on();
            break;
        default:
            LED::green_on();
            break;
    }
}

CANInterface can1(&CAND1, can1_callback);

static void cmd_elevator_set_target_position(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "e_set front_pos back_pos");
        return;
    }

    ElevatorInterface::set_position(Shell::atoi(argv[0]), Shell::atoi(argv[1]));
    chprintf(chp, "Target pos = %d, %d" SHELL_NEWLINE_STR, ElevatorInterface::target_position[0],
             ElevatorInterface::target_position[1]);
}

static void cmd_elevator_echo_target_position(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "e_echo");
        return;
    }

    chprintf(chp, "0x31B Pos = %d, V = %d, I = %d" SHELL_NEWLINE_STR,
             ElevatorInterface::elevator_wheels[0].real_position,
             ElevatorInterface::elevator_wheels[0].real_velocity,
             ElevatorInterface::elevator_wheels[0].real_current);
}

// Shell commands to control the chassis
ShellCommand elevatorInterfaceCommands[] = {
        {"e_set",  cmd_elevator_set_target_position},
        {"e_echo", cmd_elevator_echo_target_position},
        {nullptr,  nullptr}
};

class ElevatorThread : public BaseStaticThread<256> {
protected:
    void main() final {
        setName("elevator");
        ElevatorInterface::start(&can1);
        while (!shouldTerminate()) {
            ElevatorInterface::send_message();
            sleep(TIME_MS2I(100));
        }
    }
} elevatorThread;


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(elevatorInterfaceCommands);

    LED::green_off();

    can1.start_can();
    can1.start_thread(HIGHPRIO - 1);

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