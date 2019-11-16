//
// Created by liuzikai on 2019-02-24.
// Modified by LaiXinyi on 2019-7-12
//

#include "ch.hpp"
#include "hal.h"
#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "common_macro.h"
#include "buzzer_scheduler.h"

#include "engineer_chassis_interface.h"
#include "engineer_chassis_skd.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);

#if defined(BOARD_RM_2018_A)
#define STARTUP_BUTTON_PAD GPIOB
#define STARTUP_BUTTON_PIN_ID GPIOB_USER_BUTTON
#define STARTUP_BUTTON_PRESS_PAL_STATUS PAL_HIGH
#else
#error "Elevator thread is only developed for RM board 2018 A."
#endif


float target_vx = 0.0f;     // [mm/s]
float target_vy = 0.0f;     // [mm/s]
float target_w = 0.0f;      // ([degree/s], negative value for clockwise)
time_msecs_t test_end_time = 0; // [ms]

unsigned const CHASSIS_FEEDBACK_INTERVAL = 25; // [ms]

/**
 * @brief the thread for feedback data
 */
class ChassisFeedbackThread : public BaseStaticThread<1024> {
private:
    void main() final {
        setName("engineer_chassis");
        while (!shouldTerminate()) {
            Shell::printf("!cv,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f" SHELL_NEWLINE_STR,
                          TIME_I2MS(chibios_rt::System::getTime()),
                          EngineerChassisIF::motors[0].actual_velocity,
                          EngineerChassisSKD::target_velocity[0],
                          EngineerChassisIF::motors[1].actual_velocity,
                          EngineerChassisSKD::target_velocity[1],
                          EngineerChassisIF::motors[2].actual_velocity,
                          EngineerChassisSKD::target_velocity[2],
                          EngineerChassisIF::motors[3].actual_velocity,
                          EngineerChassisSKD::target_velocity[3]);
            sleep(TIME_MS2I(CHASSIS_FEEDBACK_INTERVAL));
        }
    }
} chassisFeedbackThread;


static void cmd_chassis_set_target(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 4) {
        shellUsage(chp, "c_set_target vx(mm/s) vy(mm/s) w(deg/s, + for ccw) test_time(ms)");
        return;
    }
    target_vx = Shell::atof(argv[0]);
    target_vy = Shell::atof(argv[1]);
    target_w = Shell::atof(argv[2]);

    EngineerChassisSKD::set_velocity(target_vx, target_vy, target_w);

    EngineerChassisSKD::set_test_end_time( (time_msecs_t) Shell::atoi(argv[3]));

}


static void cmd_chassis_stop(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "s");
        return;
    }
    EngineerChassisSKD::set_velocity(0,0,0);
}


static void cmd_chassis_set_vy(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "r_y vy");
        return;
    }
    EngineerChassisSKD::set_velocity(0, Shell::atoi(argv[0]),0);
}


static void cmd_chassis_set_vx(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "r_x vx");
        return;
    }
    EngineerChassisSKD::set_velocity(Shell::atoi(argv[0]),0,0);
}


static void cmd_chassis_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "c_set_params ki kp kd i_limit out_limit");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);  // echo chassis parameters error
        return;
    }

    EngineerChassisSKD::change_pid_params( {Shell::atof(argv[0]),
                                            Shell::atof(argv[1]),
                                            Shell::atof(argv[2]),
                                            Shell::atof(argv[3]),
                                            Shell::atof(argv[4])} );

    chprintf(chp, "!cps" SHELL_NEWLINE_STR); // echo chassis parameters set
}


static void cmd_chassis_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_echo_params");
        return;
    }
    chprintf(chp, "engineer chassis V to I:    ");
    EngineerChassisSKD::print_pid();
}


static void cmd_chassis_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_enable");
        return;
    }
    EngineerChassisSKD::unlock();
    chprintf(chp, "chassis unlocked");
}

static void cmd_chassis_disable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_disable");
        return;
    }
    EngineerChassisSKD::lock();
    chprintf(chp, "chassis locked");
}

ShellCommand chassisCommands[] = {
        {"c_set_params",  cmd_chassis_set_parameters},
        {"c_set_target",  cmd_chassis_set_target},
        {"s", cmd_chassis_stop},
        {"r_y", cmd_chassis_set_vy},
        {"r_x", cmd_chassis_set_vx},
        {"c_echo_params", cmd_chassis_echo_parameters},
        {"c_enable", cmd_chassis_enable},
        {"c_disable", cmd_chassis_disable},
        {nullptr,         nullptr}
};


int main(void) {

    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(chassisCommands);

    can1.start(HIGHPRIO - 1);

    EngineerChassisIF::init(&can1);
    EngineerChassisSKD::engineerChassisThread.start(NORMALPRIO);
    chassisFeedbackThread.start(NORMALPRIO - 1);

    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_intel, LOWPRIO);
    LED::green_on();

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