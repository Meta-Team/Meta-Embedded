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

static void cmd_elevator_set_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "e_set_fb index(0-3) interval[ms]");
        return;
    }

    CANTxFrame txFrame;

    // Pre-fill info of txFrame
    txFrame.IDE = CAN_IDE_STD;
    txFrame.RTR = CAN_RTR_DATA;
    txFrame.DLC = 0x08;
    for (int data_index = 0; data_index < 8; data_index++) {
        txFrame.data8[data_index] = 0x55;
    }
    txFrame.SID = (3 << 8 | ((unsigned) (Shell::atoi(argv[0]) + 1)) << 4 | 0xA); // index + 1 = CAN ID
    txFrame.data8[0] = (uint8_t)(Shell::atoi(argv[1]));
    txFrame.data8[1] = 0x00;
    can1.send_msg(&txFrame);

}



static void cmd_elevator_set_target_position(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "e_set front_pos[cm] back_pos[cm] positive for VEHICLE to DOWN");
        return;
    }
    ElevatorInterface::apply_rear_position(Shell::atof(argv[0]));
    ElevatorInterface::apply_front_position(Shell::atof(argv[1]));
    chprintf(chp, "Target pos = %f, %f" SHELL_NEWLINE_STR, Shell::atof(argv[0]), Shell::atof(argv[1]));
}

// Shell commands to control the chassis
ShellCommand elevatorInterfaceCommands[] = {
        {"e_set", cmd_elevator_set_target_position},
        {"e_set_fb", cmd_elevator_set_feedback},
        {nullptr, nullptr}
};

class ElevatorThread : public BaseStaticThread<256> {
protected:
    void main() final {
        setName("elevator");
        ElevatorInterface::init(&can1);
        while (!shouldTerminate()) {

            Shell::printf(
                    "FR[%d]: POS = %d, V = %d, FL[%d]: POS = %d, V = %d, RL[%d]: POS = %d, V = %d, RR[%d]: POS = %d, V = %d" SHELL_NEWLINE_STR,
                    ElevatorInterface::elevator_wheels[ElevatorInterface::FRONT_RIGHT].is_in_action(),
                    (int) ElevatorInterface::elevator_wheels[ElevatorInterface::FRONT_RIGHT].real_position,
                    (int) ElevatorInterface::elevator_wheels[ElevatorInterface::FRONT_RIGHT].real_velocity,
                    ElevatorInterface::elevator_wheels[ElevatorInterface::FRONT_LEFT].is_in_action(),
                    (int) ElevatorInterface::elevator_wheels[ElevatorInterface::FRONT_LEFT].real_position,
                    (int) ElevatorInterface::elevator_wheels[ElevatorInterface::FRONT_LEFT].real_velocity,
                    ElevatorInterface::elevator_wheels[ElevatorInterface::REAR_LEFT].is_in_action(),
                    (int) ElevatorInterface::elevator_wheels[ElevatorInterface::REAR_LEFT].real_position,
                    (int) ElevatorInterface::elevator_wheels[ElevatorInterface::REAR_LEFT].real_velocity,
                    ElevatorInterface::elevator_wheels[ElevatorInterface::REAR_RIGHT].is_in_action(),
                    (int) ElevatorInterface::elevator_wheels[ElevatorInterface::REAR_RIGHT].real_position,
                    (int) ElevatorInterface::elevator_wheels[ElevatorInterface::REAR_RIGHT].real_velocity);

            sleep(TIME_MS2I(2000));
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

    can1.start(HIGHPRIO - 1);

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