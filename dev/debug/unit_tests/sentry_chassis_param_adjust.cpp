//
// Created by zhukerui on 2019/6/11.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "can_interface.h"
#include "sentry_chassis_interface.h"
#include "sentry_chassis.h"

using namespace chibios_rt;

unsigned const CHASSIS_FEEDBACK_INTERVAL = 25;

bool motor_enabled[2] = {false, false};

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
                              SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].actual_angular_velocity, SentryChassisController::get_target_velocity(),
                              SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].actual_current_raw, SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].target_current);
            }
            if (enable_left_feedback) {
                Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              SYSTIME,
                              0.0f, 0.0f,
                              SentryChassisIF::motor[SentryChassisIF::MOTOR_LEFT].actual_angular_velocity, SentryChassisController::get_target_velocity(),
                              SentryChassisIF::motor[SentryChassisIF::MOTOR_LEFT].actual_current_raw, SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].target_current);
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
 * @brief set target velocity of yaw and pitch and disable pos_to_v_pid
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_set_target_velocities(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_v right left");
        return;
    }

    SentryChassisController::set_maximum_velocity(Shell::atof(argv[0]));
    SentryChassisController::motor_right_pid.clear_i_out();
    SentryChassisController::motor_left_pid.clear_i_out();
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
 * @brief set chassis common PID params
 */
static void cmd_chassis_set_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 7) {
        shellUsage(chp, "g_set_params bullet(0)/plate(1) NULL(0)/v_to_i(1) ki kp kd i_limit out_limit");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    if (*argv[0] == '0') SentryChassisController::motor_right_pid.change_parameters({Shell::atof(argv[2]),
                                                                                     Shell::atof(argv[3]),
                                                                                     Shell::atof(argv[4]),
                                                                                     Shell::atof(argv[5]),
                                                                                     Shell::atof(argv[6])});
    else
        SentryChassisController::motor_left_pid.change_parameters({Shell::atof(argv[2]),
                                                                    Shell::atof(argv[3]),
                                                                    Shell::atof(argv[4]),
                                                                    Shell::atof(argv[5]),
                                                                    Shell::atof(argv[6])});


    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

/**
 * @brief helper function for cmd_chassis_print_pid()
 */
static inline void _cmd_pid_echo_parameters(BaseSequentialStream *chp, PIDControllerBase::pid_params_t p) {
    chprintf(chp, "%f %f %f %f %f" SHELL_NEWLINE_STR, p.kp, p.ki, p.kd, p.i_limit, p.out_limit);
}

/**
 * @brief print the pid information of the specific motor
 */
static void cmd_chassis_print_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_echo_params");
        return;
    }
    chprintf(chp, "bullet v_to_i:       ");
    _cmd_pid_echo_parameters(chp, SentryChassisController::motor_right_pid.get_parameters());
    chprintf(chp, "plate v_to_i:     ");
    _cmd_pid_echo_parameters(chp, SentryChassisController::motor_left_pid.get_parameters());
}

/**
 * @brief set the target_position in the unit of cm
 */
static void cmd_chassis_set_position(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 2){
        shellUsage(chp, "g_set_angle target_position same_as_the_first_one");
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
        shellUsage(chp, "g_fix");
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
        {"g_set_v",     cmd_set_target_velocities},
        {"c_set_mode",    cmd_chassis_set_mode},
        {"g_set_params",  cmd_chassis_set_pid},
        {"g_echo_params",   cmd_chassis_print_pid},
        {"g_set_angle",   cmd_chassis_set_position},
        {"g_fix", cmd_chassis_clear_position},
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

    chassisFeedbackThread.start(NORMALPRIO - 1);
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