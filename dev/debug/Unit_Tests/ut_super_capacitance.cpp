//
// Created by Qian Chen on 5/27/21.
//
#include "ch.hpp"
#include "hal.h"

#include "debug/shell/shell.h"
#include "super_capacitor_port.h"
#include "can_interface.h"
#include "buzzer_scheduler.h"
#include "referee_interface.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_get_fb(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "get_fb");
        return;
    }
    Shell::printf("power:       %f" SHELL_NEWLINE_STR, SuperCapacitor::feedback->output_power);
    Shell::printf("current:     %f" SHELL_NEWLINE_STR, SuperCapacitor::feedback->input_current);
    Shell::printf("capvolt:     %f" SHELL_NEWLINE_STR, SuperCapacitor::feedback->capacitor_voltage);
    Shell::printf("inputvolt:   %f" SHELL_NEWLINE_STR, SuperCapacitor::feedback->input_voltage);
    Shell::printf("refPower:    %f" SHELL_NEWLINE_STR, (float) Referee::robot_state.chassis_power_limit);
}

static void cmd_set_power(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "set (power f)");
        return;
    }
    float power = Shell::atof(argv[0]);
    SuperCapacitor::set_power(power);
}


// Shell commands to ...
ShellCommand ShellCommands[] = {
        {"get_fb", cmd_get_fb},
        {"set", cmd_set_power},
        {nullptr,  nullptr}
};


// Thread to ...
class SkywalkerAdjustThread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("template");
        while (!shouldTerminate()) {
//            Shell::printf("power:       %f" SHELL_NEWLINE_STR, SuperCapacitor::feedback->output_power);
//            Shell::printf("current:     %f" SHELL_NEWLINE_STR, SuperCapacitor::feedback->input_current);
//            Shell::printf("capvolt:     %f" SHELL_NEWLINE_STR, SuperCapacitor::feedback->capacitor_voltage);
//            Shell::printf("inputvolt:   %f" SHELL_NEWLINE_STR, SuperCapacitor::feedback->input_voltage);
//            Shell::printf("refPower:    %f" SHELL_NEWLINE_STR, (float) Referee::game_robot_state.chassis_power_limit);
//            sleep(TIME_MS2I(1000));
        }
    }
} templateThread;


int main(void) {
    halInit();
    System::init();

    can1.start(NORMALPRIO+5, NORMALPRIO +6);
    can2.start(NORMALPRIO+2,NORMALPRIO+3);

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(ShellCommands);

    SuperCapacitor::init(&can2);
    templateThread.start(NORMALPRIO + 1);

    Referee::init(NORMALPRIO+7);
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