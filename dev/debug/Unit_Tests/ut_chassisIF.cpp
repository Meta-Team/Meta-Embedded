//
// Created by liuzikai on 2019-01-14.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "can_interface.h"
#include "chassis_interface.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);

/**
 * @brief echo acutal angular velocity and target current of each motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_echo(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_echo");
        return;
    }

//    chprintf(chp, "actual_velocity: FR = %.2f, FL = %.2f, BL = %.2f, BR = %.2f" SHELL_NEWLINE_STR,
//             ChassisIF::feedback[ChassisIF::FR].actual_velocity,
//             ChassisIF::feedback[ChassisIF::FL].actual_velocity,
//             ChassisIF::feedback[ChassisIF::BL].actual_velocity,
//             ChassisIF::feedback[ChassisIF::BR].actual_velocity);
//    chprintf(chp, "target_current: FR = %d, FL = %d, BL = %d, BR = %d" SHELL_NEWLINE_STR,
//             ChassisIF::target_current[ChassisIF::FR],
//             ChassisIF::target_current[ChassisIF::FL],
//             ChassisIF::target_current[ChassisIF::BL],
//             ChassisIF::target_current[ChassisIF::BR]);
}

/**
 * @brief set and send target current of each motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_set_target_currents(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 4) {
        shellUsage(chp, "c_set_current FR FL BL BR");
        return;
    }

    ChassisIF::target_current[ChassisIF::FR] = Shell::atoi(argv[0]);
    ChassisIF::target_current[ChassisIF::FL] = Shell::atoi(argv[1]);
    ChassisIF::target_current[ChassisIF::BL] = Shell::atoi(argv[2]);
    ChassisIF::target_current[ChassisIF::BR] = Shell::atoi(argv[3]);
    chprintf(chp, "target_current: FR = %d, FL = %d, BL = %d, BR = %d" SHELL_NEWLINE_STR,
             ChassisIF::target_current[ChassisIF::FR],
             ChassisIF::target_current[ChassisIF::FL],
             ChassisIF::target_current[ChassisIF::BL],
             ChassisIF::target_current[ChassisIF::BR]);

    ChassisIF::send_chassis_currents();
    chprintf(chp, "Chassis target_current sent" SHELL_NEWLINE_STR);
}

// Shell commands to control the chassis
ShellCommand chassisCommands[] = {
        {"c_echo", cmd_chassis_echo},
        {"c_set_current",    cmd_chassis_set_target_currents},
        {nullptr,    nullptr}
};

class GimbalThread : public BaseStaticThread<256> {
protected:
    void main() final {
        setName("chassis");
        while (!shouldTerminate()) {

            ChassisIF::send_chassis_currents();

            sleep(TIME_MS2I(100));
        }
    }
} chassisThread;


int main(void) {
    halInit();
    System::init();
    LED::green_off();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(chassisCommands);

    can1.start(HIGHPRIO - 1);
    ChassisIF::init(&can1);

    chassisThread.start(NORMALPRIO);

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
