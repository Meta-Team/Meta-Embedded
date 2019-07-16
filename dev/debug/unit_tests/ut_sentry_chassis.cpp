//
// Created by liuzikai on 2019-04-12.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "can_interface.h"
#include "sentry_chassis_interface.h"
#include "sentry_chassis_scheduler.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);
bool printPosition = false;
bool printCurrent = false;
bool printVelocity = false;

/**
 * @brief enable the chassis
 */
static void cmd_chassis_enable(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_enable");
        return;
    }
    SentryChassisSKD::enable = true;
}

/**
 * @brief disable the chassis
 */
static void cmd_chassis_disable(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_disable");
        return;
    }
    SentryChassisSKD::enable = false;
}

/**
 * @brief set the mode for the sentry
 * @attention test_mode is true for testing, and is false for automatically driving
 */
static void cmd_chassis_set_mode(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1 && argc != 2) {
        shellUsage(chp, "c_set_mode stop_mode(0)/one_step_mode(2)/auto_mode(3) radius(10~50)");
        return;
    }
    switch (*argv[0]){
        case ('2'):
            SentryChassisSKD::set_mode(SentryChassisSKD::ONE_STEP_MODE);
            break;
        case ('3'):
            if (argc != 2) {
                shellUsage(chp, "c_set_mode auto_mode(3) radius(10~50)");
                return;
            }
            SentryChassisSKD::set_mode(SentryChassisSKD::SHUTTLED_MODE, Shell::atof(argv[1]));
            break;
        default:
            SentryChassisSKD::set_mode(SentryChassisSKD::STOP_MODE);
    }
}

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

    chprintf(chp, "actual_velocity: LEFT = %.2f, RIGHT = %.2f" SHELL_NEWLINE_STR,
             SentryChassisIF::motor[SentryChassisIF::MOTOR_LEFT].actual_angular_velocity,
             SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].actual_angular_velocity);
    chprintf(chp, "target_current: LEFT = %d, RIGHT = %d" SHELL_NEWLINE_STR,
             SentryChassisIF::motor[SentryChassisIF::MOTOR_LEFT].target_current,
             SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].target_current);
}

/**
 * @brief set and send target current of each motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_set_target_currents(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "c_set_current LEFT RIGHT");
        return;
    }

    SentryChassisIF::motor[SentryChassisIF::MOTOR_LEFT].target_current = Shell::atoi(argv[0]);
    SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].target_current = Shell::atoi(argv[1]);
    chprintf(chp, "target_current: LEFT = %d, RIGHT = %d" SHELL_NEWLINE_STR,
             SentryChassisIF::motor[SentryChassisIF::MOTOR_LEFT].target_current,
             SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].target_current);
    SentryChassisIF::send_currents();
    chprintf(chp, "Chassis target_current sent" SHELL_NEWLINE_STR);
}

/**
 * @brief set chassis common PID params
 */
static void cmd_chassis_set_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "c_set_pid ki kp kd i_limit out_limit");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);  // echo chassis parameters error
        return;
    }

    SentryChassisSKD::change_v_to_i_pid(Shell::atof(argv[0]),
                                               Shell::atof(argv[1]),
                                               Shell::atof(argv[2]),
                                               Shell::atof(argv[3]),
                                               Shell::atof(argv[4]));

    chprintf(chp, "!cps" SHELL_NEWLINE_STR); // echo chassis parameters set
}

/**
 * @brief print the pid information of the specific motor
 */
static void cmd_chassis_print_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_pid");
        return;
    }
    //SentryChassisSKD::print_pid_params();
}

/**
 * @brief set the target_position in the unit of cm
 */
static void cmd_chassis_set_position(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1){
        shellUsage(chp, "c_set_pos target_position");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    SentryChassisSKD::set_destination(Shell::atof(argv[0]));
}

/**
 * @brief clear the present and target position
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_clear_position(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0){
        shellUsage(chp, "c_clear");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    SentryChassisSKD::set_origin();
}

static void cmd_chassis_print_position(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0){
        shellUsage(chp, "c_pos");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    printPosition = !printPosition;
}

static void cmd_chassis_print_current(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0){
        shellUsage(chp, "c_cur");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    printCurrent = !printCurrent;
}

static void cmd_chassis_print_velocity(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0){
        shellUsage(chp, "c_v");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    printVelocity = !printVelocity;
}

static void cmd_chassis_set_variable_speed_mode(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1){
        shellUsage(chp, "c_change_v variable_speed(0/1)");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    SentryChassisSKD::change_speed_mode(*argv[0]=='1');
}
// Shell commands to control the chassis
ShellCommand chassisCommands[] = {
        {"c_enable",   cmd_chassis_enable},
        {"c_disable",   cmd_chassis_disable},
        {"c_set_mode",    cmd_chassis_set_mode},
        {"c_echo", cmd_chassis_echo},
        {"c_set_current",   cmd_chassis_set_target_currents},
        {"c_set_pid",  cmd_chassis_set_pid},
        {"c_pid",   cmd_chassis_print_pid},
        {"c_set_pos",   cmd_chassis_set_position},
        {"c_clear", cmd_chassis_clear_position},
        {"c_pos", cmd_chassis_print_position},
        {"c_cur", cmd_chassis_print_current},
        {"c_v", cmd_chassis_print_velocity},
        {"c_change_v", cmd_chassis_set_variable_speed_mode},
        {nullptr,    nullptr}
};


class SentryChassisThread : public BaseStaticThread<256> {
protected:
    void main() final {
        setName("chassis");
        while (!shouldTerminate()) {
            if(printPosition)
                SentryChassisSKD::print_position();

            if(printCurrent)
                SentryChassisSKD::print_current();

            if(printVelocity)
                SentryChassisSKD::print_velocity();

            SentryChassisSKD::update_target_current();

            SentryChassisSKD::send_currents();

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
    SentryChassisSKD::init(&can1);

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
