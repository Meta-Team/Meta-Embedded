//
// Created by liuzikai on 2019-02-24.
// Modified by Zhu Kerui on 2019-07-13
//

#include "ch.hpp"
#include "hal.h"
#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "common_macro.h"
#include "buzzer_scheduler.h"

#include "robotic_arm_interface.h"
#include "robotic_arm_skd.h"

#if defined(BOARD_RM_2018_A)
#define STARTUP_BUTTON_PAD GPIOB
#define STARTUP_BUTTON_PIN_ID GPIOB_USER_BUTTON
#define STARTUP_BUTTON_PRESS_PAL_STATUS PAL_HIGH
#else
#error "Robitic Arm Unit Test is only developed for RM board 2018 A."
#endif

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

//static void cmd_robotic_clamp_action(BaseSequentialStream *chp, int argc, char *argv[]) {
//    (void) argv;
//    if (argc != 1) {
//        shellUsage(chp, "clamp 0(relax)/1(clamped)");
//        return;
//    }
//    if (Shell::atoi(argv[0]) == 0) {
//        RoboticArmSKD::set_clamp_action(RoboticArmSKD::CLAMP_RELAX);
//    } else {
//        RoboticArmSKD::set_clamp_action(RoboticArmSKD::CLAMP_CLAMPED);
//    }
//}
//
static void cmd_robotic_arm_action(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "rotate 0(pull_back)/1(stretch_out)");
        return;
    }
    if (Shell::atoi(argv[0])) RoboticArmSKD::stretch_out();
    else RoboticArmSKD::pull_back();
}

static void cmd_clamp_test(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s (emergency stop)");
        return;
    }
    RoboticArmSKD::change_clamp();
}

static void cmd_power_on(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc!=0) {
        shellUsage(chp, "fail");
        return;
    }
    palSetPad(GPIOH, POWER_PAD);
    RoboticArmSKD::state = RoboticArmSKD::COLLECT_BULLET;
    RoboticArmSKD::bullet_state = RoboticArmSKD::BOX_CLAMPED;
}

static void cmd_power_off(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc!=0) {
        shellUsage(chp, "fail");
        return;
    }
    palClearPad(GPIOH, POWER_PAD);
    RoboticArmSKD::state = RoboticArmSKD::NORMAL;
}

static void cmd_robotic_arm_emergency_stop(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s (emergency stop)");
        return;
    }
    RoboticArmSKD::released = true;
    chprintf(chp, "EMERGENCY STOP" SHELL_NEWLINE_STR);
}

void cmd_robotic_arm_set_v2i(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "set_v2i kp ki kd i_limit out_limit");
        return;
    }
    RoboticArmSKD::v2i_pid.change_parameters({Shell::atof(argv[0]),
                                              Shell::atof(argv[1]),
                                              Shell::atof(argv[2]),
                                              Shell::atof(argv[3]),
                                              Shell::atof(argv[4])});
    LOG("pass!");
}

static void cmd_robotic_arm_next(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc!=0) {
        shellUsage(chp, "fail");
        return;
    }
    RoboticArmSKD::next_step();
}

static void cmd_robotic_arm_prev(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc!=0) {
        shellUsage(chp, "fail");
        return;
    }
    RoboticArmSKD::prev_step();
}

static void cmd_extend(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc!=0) {
        shellUsage(chp, "fail");
        return;
    }
    RoboticArmSKD::set_digital_status(RoboticArmSKD::extend_state, EXTEND_PAD, RoboticArmSKD::LOW_STATUS);
}

static void cmd_extend_2(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc!=0) {
        shellUsage(chp, "fail");
        return;
    }
    RoboticArmSKD::set_digital_status(RoboticArmSKD::extend_state, EXTEND_PAD, RoboticArmSKD::HIGH_STATUS);
}

ShellCommand roboticArmCommands[] = {
        {"clamp", cmd_clamp_test},
        {"rotate", cmd_robotic_arm_action},
        {"extend", cmd_extend},
        {"extend2", cmd_extend_2},
        {"s", cmd_robotic_arm_emergency_stop},
        {"set_v2i", cmd_robotic_arm_set_v2i},
        {"next", cmd_robotic_arm_next},
        {"prev", cmd_robotic_arm_prev},
        {"on", cmd_power_on},
        {"off", cmd_power_off},
        {nullptr, nullptr}
};

class FeedbackThread : public chibios_rt::BaseStaticThread<512> {
    void main() final {
        setName("robotic_arm_fb");
        while(!shouldTerminate()) {
            LOG("rotation motor pos = %f rotation motor v = %f" SHELL_NEWLINE_STR, RoboticArmIF::present_angle, RoboticArmIF::present_velocity);
            sleep(TIME_MS2I(200));
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
    can2.start(HIGHPRIO - 2);
    RoboticArmIF::init(&can2);
    RoboticArmSKD::roboticArmThread.start(HIGHPRIO - 3);

    while (palReadPad(STARTUP_BUTTON_PAD, STARTUP_BUTTON_PIN_ID) != STARTUP_BUTTON_PRESS_PAL_STATUS) {
        // Wait for the button to be pressed
        LED::green_toggle();
        chThdSleepMilliseconds(300);
    }

    feedbackThread.start(NORMALPRIO);

    BuzzerSKD::play_sound(BuzzerSKD::sound_startup, LOWPRIO);

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