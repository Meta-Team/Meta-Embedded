//
// Created by liuzikai on 2019-02-24.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "can_interface.h"
#include "elevator_interface.h"

#include "vehicle/engineer/elevator_thread.h"

CANInterface can1(&CAND1);
ElevatorThread elevatorThread;

#if defined(BOARD_RM_2018_A)
#define STARTUP_BUTTON_PAD GPIOB
#define STARTUP_BUTTON_PIN_ID GPIOB_USER_BUTTON
#define STARTUP_BUTTON_PRESS_PAL_STATUS PAL_HIGH
#else
#error "Elevator thread is only developed for RM board 2018 A."
#endif

static void cmd_elevator_up(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "engi_up");
        return;
    }
    elevatorThread.start_up_actions(NORMALPRIO + 1);
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
        {"s",       cmd_elevator_emergency_stop},
        {nullptr,   nullptr}
};

class ChassisThread : public chibios_rt::BaseStaticThread<512> {

    static constexpr int chassis_thread_interval = 20; // ms

    void main() final {
        setName("chassis");
        while (!shouldTerminate()) {


            // Pack the actual velocity into an array
            float measured_velocity[4];
            for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                measured_velocity[i] = ChassisInterface::motor[i].actual_angular_velocity;
            }

            // Perform calculation
            ChassisController::calc(measured_velocity, 0, elevatorThread.get_chassis_target_vy(), 0);

            // Pass the target current to interface
            for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                ChassisInterface::motor[i].target_current = (int) ChassisController::motor[i].target_current;
            }

            ChassisInterface::send_chassis_currents();

            sleep(TIME_MS2I(chassis_thread_interval));
        }
    }
} chassisThread;

int main(void) {
    halInit();
    chibios_rt::System::init();

    Shell::start(HIGHPRIO);
    Shell::addCommands(elevatorInterfaceCommands);

    LED::red_off();
    LED::green_off();

    can1.start(HIGHPRIO - 1);
    ChassisInterface::init(&can1);
    ElevatorInterface::init(&can1);

    ChassisController::change_pid_params(33, 0.49, 2.4, 2000, 5000);
    chassisThread.start(NORMALPRIO);

    while (palReadPad(STARTUP_BUTTON_PAD, STARTUP_BUTTON_PIN_ID) != STARTUP_BUTTON_PRESS_PAL_STATUS) {
        // Wait for the button to be pressed
        LED::green_toggle();
        chThdSleepMilliseconds(300);
    }

    elevatorThread.start_up_actions(NORMALPRIO + 1);


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