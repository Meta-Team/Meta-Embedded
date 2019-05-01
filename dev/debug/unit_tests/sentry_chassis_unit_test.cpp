//
// Created by liuzikai on 2019-04-12.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "sentry_chassis_interface.h"
#include "sentry_chassis_calculator.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);

/**
 * @brief enable the chassis
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_enable(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_enable");
        return;
    }
    SentryChassisController::enable = true;
}

/**
 * @brief disable the chassis
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_disable(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_disable");
        return;
    }
    SentryChassisController::enable = false;
}

/**
 * @brief set the mode for the sentry
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_test_mode(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "c_test_mode test_mode(0/1)");
        return;
    }
    SentryChassisController::test_mode = (*argv[0] == '0');
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

    chprintf(chp, "actual_angular_velocity: LEFT = %.2f, RIGHT = %.2f" SHELL_NEWLINE_STR,
             SentryChassis::motor[SentryChassis::MOTOR_LEFT].actual_angular_velocity,
             SentryChassis::motor[SentryChassis::MOTOR_RIGHT].actual_angular_velocity);
    chprintf(chp, "target_current: LEFT = %d, RIGHT = %d" SHELL_NEWLINE_STR,
             SentryChassis::motor[SentryChassis::MOTOR_LEFT].target_current,
             SentryChassis::motor[SentryChassis::MOTOR_RIGHT].target_current);
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

    SentryChassis::motor[SentryChassis::MOTOR_LEFT].target_current = Shell::atoi(argv[0]);
    SentryChassis::motor[SentryChassis::MOTOR_RIGHT].target_current = Shell::atoi(argv[1]);
    chprintf(chp, "target_current: LEFT = %d, RIGHT = %d" SHELL_NEWLINE_STR,
             SentryChassis::motor[SentryChassis::MOTOR_LEFT].target_current,
             SentryChassis::motor[SentryChassis::MOTOR_RIGHT].target_current);
    SentryChassis::send_currents();
    chprintf(chp, "Chassis target_current sent" SHELL_NEWLINE_STR);
}

/**
 * @brief set chassis common PID params
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_set_dist_to_v_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "c_set_dist_to_v_params ki kp kd i_limit out_limit");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);  // echo chassis parameters error
        return;
    }


    SentryChassisController::change_dist_to_v_pid(Shell::atof(argv[0]),
                                         Shell::atof(argv[1]),
                                         Shell::atof(argv[2]),
                                         Shell::atof(argv[3]),
                                         Shell::atof(argv[4]));
    chprintf(chp, "!cps" SHELL_NEWLINE_STR); // echo chassis parameters set
}

/**
 * @brief set chassis common PID params
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_set_v_to_i_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "c_set_v_to_i_params ki kp kd i_limit out_limit");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);  // echo chassis parameters error
        return;
    }


    SentryChassisController::change_v_to_i_pid(Shell::atof(argv[0]),
                                                  Shell::atof(argv[1]),
                                                  Shell::atof(argv[2]),
                                                  Shell::atof(argv[3]),
                                                  Shell::atof(argv[4]));
    chprintf(chp, "!cps" SHELL_NEWLINE_STR); // echo chassis parameters set
}
/**
 * @brief print the pid information of the specific motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_echo_pid_params(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "c_echo_pid_params motor_id");
        return;
    }
    int motor_id = Shell::atoi(argv[0]);
    if(motor_id != 0 && motor_id != 1){
        shellUsage(chp, "wrong motor_id");
        return;
    }
    SentryChassisController::print_pid_params(chp, motor_id);
}

/**
 * @brief set the target_position in the unit of cm
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_target_position(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1){
        shellUsage(chp, "c_target_position target_position");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    SentryChassisController::move_certain_dist(Shell::atof(argv[0]));
}

static void cmd_chassis_clear_position(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0){
        shellUsage(chp, "c_clear_position");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    SentryChassisController::reset_present_position();
}

// Shell commands to control the chassis
ShellCommand chassisCommands[] = {
        {"c_enable",   cmd_chassis_enable},
        {"c_disable",   cmd_chassis_disable},
        {"cs_test_mode",    cmd_chassis_test_mode},
        {"c_echo", cmd_chassis_echo},
        {"c_set_current",   cmd_chassis_set_target_currents},
        {"c_set_dist_to_v_params",  cmd_chassis_set_dist_to_v_parameters},
        {"c_set_v_to_i_params", cmd_chassis_set_v_to_i_parameters},
        {"c_echo_pid_params",   cmd_chassis_echo_pid_params},
        {"c_target_position",   cmd_chassis_target_position},
        {"c_clear_position", cmd_chassis_clear_position},
        {nullptr,    nullptr}
};


class SentryChassisThread : public BaseStaticThread<256> {
protected:
    void main() final {
        setName("chassis");
        while (!shouldTerminate()) {

            SentryChassisController::update_present_data();
            /*
            if (!SentryChassisController::test_mode){}
            */
            SentryChassisController::update_target_current();
            SentryChassisController::send_currents();
            //SentryChassis::send_currents();

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
    SentryChassisController::init_controller(&can1);

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
