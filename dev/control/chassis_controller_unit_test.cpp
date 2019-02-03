//
// Created by liuzikai on 2019-02-03.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "chassis_interface.h"
#include "chassis_controller.h"

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

float target_vx = 0.0f; // (mm/s)
float target_vy = 0.0f; // (mm/s)
float target_w = 0.0f; // (degree/s, negative value for clockwise)

int test_time = 1000; // (ms)

int const chassis_thread_interval = 20; // ms

/**
 * @brief set chassis controller target
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_set_target(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 4) {
        shellUsage(chp, "c_set_target vx(mm/s) vy(mm/s) w(deg/s, + for ccw) test_time(ms)");
        return;
    }

    target_vx = Shell::atof(argv[0]);
    target_vy = Shell::atof(argv[1]);
    target_w = Shell::atof(argv[2]);
    test_time = Shell::atoi(argv[3]);
}

/**
 * @brief set chassis common PID params
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "c_set_params ki kp kd i_limit out_limit");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);  // echo chassis parameters error
        return;
    }


    ChassisController::change_pid_params(Shell::atof(argv[0]),
                                     Shell::atof(argv[1]),
                                     Shell::atof(argv[2]),
                                     Shell::atof(argv[3]),
                                     Shell::atof(argv[4]));
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        ChassisController::motor[i].pid.clear_i_out();
    }
    chprintf(chp, "!cps" SHELL_NEWLINE_STR); // echo chassis parameters set
}

// Shell commands to control the chassis
ShellCommand chassisInterfaceCommands[] = {
        {"c_set_params", cmd_chassis_set_parameters},
        {"c_set_target", cmd_chassis_set_target},
        {nullptr,    nullptr}
};

class GimbalThread : public BaseStaticThread<256> {
protected:
    void main() final {
        setName("chassis");
        while (!shouldTerminate()) {

            ChassisInterface::send_chassis_currents();

            sleep(TIME_MS2I(chassis_thread_interval));
        }
    }
} chassisThread;


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(chassisInterfaceCommands);


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
