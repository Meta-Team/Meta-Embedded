//
// Created by liuzikai on 2019-01-14.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "chassis_interface.h"

using namespace chibios_rt;

/**
 * @brief callback function for CAN1
 * @param rxmsg
 */
static void can1_callback(CANRxFrame *rxmsg) {
    switch (rxmsg->SID) {
        case 0x201:
        case 0x202:
        case 0x203:
        case 0x204:
            ChassisInterface::process_chassis_feedback(rxmsg);
            break;
        default:
            break;
    }
}

CANInterface can1(&CAND1, can1_callback);

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

    chprintf(chp, "actual_angular_velocity: FR = %.2f, FL = %.2f, BL = %.2f, BR = %.2f" SHELL_NEWLINE_STR,
             ChassisInterface::motor[CHASSIS_FR].actual_angular_velocity,
             ChassisInterface::motor[CHASSIS_FL].actual_angular_velocity,
             ChassisInterface::motor[CHASSIS_BL].actual_angular_velocity,
             ChassisInterface::motor[CHASSIS_BR].actual_angular_velocity);
    chprintf(chp, "target_current: FR = %d, FL = %d, BL = %d, BR = %d" SHELL_NEWLINE_STR,
             ChassisInterface::motor[CHASSIS_FR].target_current,
             ChassisInterface::motor[CHASSIS_FL].target_current,
             ChassisInterface::motor[CHASSIS_BL].target_current,
             ChassisInterface::motor[CHASSIS_BR].target_current);
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

    ChassisInterface::motor[CHASSIS_FR].target_current = Shell::atoi(argv[0]);
    ChassisInterface::motor[CHASSIS_FL].target_current = Shell::atoi(argv[1]);
    ChassisInterface::motor[CHASSIS_BL].target_current = Shell::atoi(argv[2]);
    ChassisInterface::motor[CHASSIS_BR].target_current = Shell::atoi(argv[3]);
    chprintf(chp, "target_current: FR = %d, FL = %d, BL = %d, BR = %d" SHELL_NEWLINE_STR,
             ChassisInterface::motor[CHASSIS_FR].target_current,
             ChassisInterface::motor[CHASSIS_FL].target_current,
             ChassisInterface::motor[CHASSIS_BL].target_current,
             ChassisInterface::motor[CHASSIS_BR].target_current);

    ChassisInterface::send_chassis_currents();
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

            ChassisInterface::send_chassis_currents();

            sleep(TIME_MS2I(100));
        }
    }
} chassisThread;


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(chassisCommands);


    can1.start_can();
    can1.start_thread(HIGHPRIO - 1);
    ChassisInterface::set_can_interface(&can1);

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
