//
// Created by zhukerui on 2019/6/11.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
//#include "debug/shell/shell.h"
#include "can_interface.h"
#include "sentry_chassis_interface.h"
#include "sentry_chassis_calculator.h"

using namespace chibios_rt;

unsigned const CHASSIS_FEEDBACK_INTERVAL = 25;

bool motor_enabled[2] = {false, false};

float target_v[2] = {0, 0};

CANInterface can1(&CAND1);
bool printPosition = false;
bool printCurrent = false;
bool printVelocity = false;
bool testCurrent = false;

class ChassisFeedbackThread : public chibios_rt::BaseStaticThread<1024> {

public:

    bool enable_right_feedback = false;
    bool enable_left_feedback = false;

private:

    void main() final {

        setName("chassis_fb");

        while (!shouldTerminate()) {

            if (enable_right_feedback) {
                Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              SYSTIME,
                              0.0f, 0.0f,
                              SentryChassis::motor[SentryChassis::MOTOR_RIGHT].actual_angular_velocity, target_v[0],
                              SentryChassis::motor[SentryChassis::MOTOR_RIGHT].actual_current_raw, SentryChassis::motor[SentryChassis::MOTOR_RIGHT].target_current);
            }
            if (enable_left_feedback) {
                Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              SYSTIME,
                              0.0f, 0.0f,
                              SentryChassis::motor[SentryChassis::MOTOR_LEFT].actual_angular_velocity, target_v[1],
                              SentryChassis::motor[SentryChassis::MOTOR_LEFT].actual_current_raw, SentryChassis::motor[SentryChassis::MOTOR_RIGHT].target_current);
            }

            sleep(TIME_MS2I(CHASSIS_FEEDBACK_INTERVAL));
        }
    }

}chassisFeedbackThread;


/**
 * @brief enable the chassis
 */
static void cmd_chassis_enable(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_enable left(0/1) right(0/1)");
        return;
    }
    SentryChassisController::enable = (*argv[0] - '0') || (*argv[1] - '0');
}

static void cmd_chassis_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1')) {
        shellUsage(chp, "g_enable_fb right(0/1) left(0/1)");
        return;
    }
    chassisFeedbackThread.enable_right_feedback = *argv[0] - '0';
    chassisFeedbackThread.enable_left_feedback = *argv[1] - '0';
}

/**
 * @brief set the mode for the sentry
 * @attention test_mode is true for testing, and is false for automatically driving
 */
static void cmd_chassis_set_mode(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "c_set_mode stop_mode(0)/one_step_mode(1)/auto_mode(2)");
        return;
    }
    switch (*argv[0]){
        case ('1'):
            SentryChassisController::set_mode(SentryChassisController::ONE_STEP_MODE);
            break;
        case ('2'):
            SentryChassisController::set_mode(SentryChassisController::AUTO_MODE);
            break;
        default:
            SentryChassisController::set_mode(SentryChassisController::STOP_MODE);
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
 */
static void cmd_chassis_set_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "c_set_pid ki kp kd i_limit out_limit");
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
 */
static void cmd_chassis_print_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "c_pid motor_id");
        return;
    }
    int motor_id = Shell::atoi(argv[0]);
    if(motor_id != 0 && motor_id != 1){
        shellUsage(chp, "wrong motor_id");
        return;
    }
    SentryChassisController::print_pid_params(motor_id);
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
    SentryChassisController::set_destination(Shell::atof(argv[0]));
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
    SentryChassisController::clear_position();
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

static void cmd_chassis_test_current(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0){
        shellUsage(chp, "c_testC");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    testCurrent = !testCurrent;
}
// Shell commands to control the chassis
ShellCommand chassisCommands[] = {
        {"g_enable",   cmd_chassis_enable},
        {"g_enable_fb", cmd_chassis_enable_feedback},
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
        {"c_testc", cmd_chassis_test_current},
        {nullptr,    nullptr}
};


class SentryChassisThread : public BaseStaticThread<256> {
protected:
    void main() final {
        setName("chassis");
        while (!shouldTerminate()) {

            if(testCurrent){
                // In testCurrent mode, we send a constant current to the motor without processing any feedback
                // This is a fundamental test, check how the motors respond to the given current
                if(SentryChassisController::enable){
                    SentryChassisController::motor[0].target_current = 1000;
                    SentryChassisController::motor[1].target_current = 1000;
                }else{
                    SentryChassisController::motor[0].target_current = 0;
                    SentryChassisController::motor[1].target_current = 0;
                }
            }else{
                SentryChassisController::update_present_data();

                if(printPosition)
                    SentryChassisController::print_position();

                if(printCurrent)
                    SentryChassisController::print_current();

                if(printVelocity)
                    SentryChassisController::print_velocity();

                SentryChassisController::update_target_current();
            }
            SentryChassisController::send_currents();

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