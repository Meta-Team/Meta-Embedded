//
// Created by 钱晨 on 11/28/20.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "can_interface.h"
#include "pid_controller.hpp"
#include "vehicle/infantry/vehicle_infantry.h"
#include "remote_interpreter.h"

CANInterface can1(&CAND1);

int *FW_L_CURRENT;
int *FW_R_CURRENT;
float max_speed;

CANInterface::motor_feedback_t *FW_L_FB;
CANInterface::motor_feedback_t *FW_R_FB;

PIDController FW_L_v2i_PID;
PIDController FW_R_v2i_PID;
// Other headers here

using namespace chibios_rt;

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_emergency_stop(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "stop");
        return;
    }
    chprintf(chp, "Stoped." SHELL_NEWLINE_STR);
}

static void cmd_set_max_speed(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "set (float)");
        return;
    }
    max_speed = Shell::atof(argv[0]);
}

// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"stop", cmd_emergency_stop},
        {"set", cmd_set_max_speed},
        {nullptr,    nullptr}
};


// Thread to ...
class pidThread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("pidThread");
        float target_speed;
        while (!shouldTerminate()) {
            target_speed = Remote::rc.ch3 * max_speed;
            if (ABS_IN_RANGE(Remote::rc.ch3, 0.2)) {
                LED::green_on();
            } else {
                LED::green_off();
            }
            if(Remote::rc.s1 == Remote::S_MIDDLE) {
                *FW_L_CURRENT = 0;
                *FW_R_CURRENT = 0;
            } else {
                *FW_L_CURRENT = FW_L_v2i_PID.calc(FW_L_FB->actual_velocity, target_speed);
                *FW_R_CURRENT = FW_L_v2i_PID.calc(FW_R_FB->actual_velocity, target_speed);
            }
            sleep(TIME_MS2I(5));
        }
    }
} PIDThread;


int main(void) {
    halInit();
    System::init();

    can1.set_motor_type(2, CANInterface::M3508);
    can1.set_motor_type(3, CANInterface::M3508);

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);

    Remote::start();
    can1.start(NORMALPRIO + 2, NORMALPRIO + 3);

    FW_L_CURRENT = can1.register_target_current_address(2);
    FW_R_CURRENT = can1.register_target_current_address(3);

    FW_L_FB = can1.register_feedback_address(2);
    FW_R_FB = can1.register_feedback_address(3);

    FW_L_v2i_PID.change_parameters(SHOOT_PID_FW_LEFT_V2I_PARAMS);
    FW_R_v2i_PID.change_parameters(SHOOT_PID_FW_RIGHT_V2I_PARAMS);

    max_speed = 3000.0;

    PIDThread.start(NORMALPRIO + 1);

#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}