//
// Created by liuzikai on 2019-02-24.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "robotic_arm.h"

#include "vehicle/engineer/robotic_arm_thread.h"

#include "buzzer.h"

#if defined(BOARD_RM_2018_A)
#define STARTUP_BUTTON_PAD GPIOB
#define STARTUP_BUTTON_PIN_ID GPIOB_USER_BUTTON
#define STARTUP_BUTTON_PRESS_PAL_STATUS PAL_HIGH
#else
#error "Robitic Arm Unit Test is only developed for RM board 2018 A."
#endif

CANInterface can1(&CAND1);
RoboticArmThread roboticArmThread;

static void cmd_robotic_clamp_action(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "clamp 0(relax)/1(clamped)");
        return;
    }
    if (Shell::atoi(argv[0]) == 0) {
        RoboticArm::clamp_action(RoboticArm::CLAMP_RELAX);
    } else {
        RoboticArm::clamp_action(RoboticArm::CLAMP_CLAMPED);
    }
}

static void cmd_robotic_arm_action(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "engi_fetch");
        return;
    }
    int ret = roboticArmThread.start_initial_outward(NORMALPRIO);
    chprintf(chp, "Start up action = %d" SHELL_NEWLINE_STR, ret);
}

static void cmd_robotic_arm_emergency_stop(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s (emergency stop)");
        return;
    }
    roboticArmThread.emergency_stop();
    chprintf(chp, "EMERGENCY STOP" SHELL_NEWLINE_STR);
}

ShellCommand roboticArmCommands[] = {
        {"clamp", cmd_robotic_clamp_action},
        {"engi_fetch", cmd_robotic_arm_action},
        {"s", cmd_robotic_arm_emergency_stop},
        {nullptr, nullptr}
};

class FeedbackThread : public chibios_rt::BaseStaticThread<512> {
    void main() final {
        setName("robotic_arm_fb");
        while(!shouldTerminate()) {
            Shell::printf("rotation motor pos = %f" SHELL_NEWLINE_STR, RoboticArm::get_motor_actual_angle());
            sleep(TIME_MS2I(2000));
        }
    }
} feedbackThread;

int main(void) {
    halInit();
    chibios_rt::System::init();

    Shell::start(HIGHPRIO);
    Shell::addCommands(roboticArmCommands);

    LED::red_off();
    LED::green_off();

    can1.start(HIGHPRIO - 1);
    RoboticArm::init(&can1);

    chThdSleepMilliseconds(2000);
    RoboticArm::reset_front_angle();

    while (palReadPad(STARTUP_BUTTON_PAD, STARTUP_BUTTON_PIN_ID) != STARTUP_BUTTON_PRESS_PAL_STATUS) {
        // Wait for the button to be pressed
        LED::green_toggle();
        chThdSleepMilliseconds(300);
    }

//    feedbackThread.start(NORMALPRIO);

    roboticArmThread.start_initial_outward(NORMALPRIO);

    Buzzer::play_sound(Buzzer::sound_startup, LOWPRIO);

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