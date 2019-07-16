//
// Created by zhukerui on 2019/6/11.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "can_interface.h"
#include "sentry_chassis_interface.h"
#include "sentry_chassis_scheduler.h"
#include "referee_interface.h"

using namespace chibios_rt;

unsigned const CHASSIS_FEEDBACK_INTERVAL = 25;

bool motor_enabled[2] = {false, false};

class ChassisFeedbackThread : public chibios_rt::BaseStaticThread<1024> {

public:

    bool enable_right_feedback = false;
    bool enable_left_feedback = false;

private:

    void main() final {

        setName("chassis_fb");

        while (!shouldTerminate()) {

            if (enable_right_feedback) {
                Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%f,%f" SHELL_NEWLINE_STR,
                              SYSTIME,
                              //SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].motor_present_position, SentryChassisIF::target_position,
                              Referee::power_heat_data.chassis_power, SentryChassisIF::power_limit,
                              SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].motor_present_velocity, SentryChassisIF::target_velocity,
                              //Referee::power_heat_data.chassis_power, 20
                              SentryChassisIF::motor[SentryChassisIF::MOTOR_LEFT].motor_present_position, SentryChassisIF::target_position);
            }
            if (enable_left_feedback) {
                Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%f,%f" SHELL_NEWLINE_STR,
                              SYSTIME,
                              //SentryChassisIF::motor[SentryChassisIF::MOTOR_LEFT].motor_present_position, SentryChassisIF::target_position,
                              Referee::power_heat_data.chassis_power, SentryChassisIF::power_limit,
                              SentryChassisIF::motor[SentryChassisIF::MOTOR_LEFT].motor_present_velocity, SentryChassisIF::target_velocity,
                              //Referee::power_heat_data.chassis_power, 20
                              SentryChassisIF::motor[SentryChassisIF::MOTOR_LEFT].motor_present_position, SentryChassisIF::target_position);
            }

            sleep(TIME_MS2I(CHASSIS_FEEDBACK_INTERVAL));
        }
    }

}chassisFeedbackThread;

/**
     * @brief debug helper function
     * @param pid_id -- the same as "set_pid"
     */
static void print_pid(int pid_id){
    if (pid_id == 0){
        PIDControllerBase::pid_params_t to_print = left_v2i_pid.get_parameters();
        LOG("%f %f %f %f %f", to_print.kp, to_print.ki, to_print.kd, to_print.i_limit, to_print.out_limit);
    } else if (pid_id == 1){
        PIDControllerBase::pid_params_t to_print = sentry_a2v_pid.get_parameters();
        LOG("%f %f %f %f %f", to_print.kp, to_print.ki, to_print.kd, to_print.i_limit, to_print.out_limit);
    } else if (pid_id == 2){
        PIDControllerBase::pid_params_t to_print = sentry_POM_pid.get_parameters();
        LOG("%f %f %f %f %f", to_print.kp, to_print.ki, to_print.kd, to_print.i_limit, to_print.out_limit);
    }
}

// Debug options
static bool printPosition;
static bool printCurrent;
static bool printVelocity;
static bool printPower;

void SChassisSKD::SentryChassisThread::main() {

    setName("SentryChassis");
    SChassisSKD::start();

    while (!shouldTerminate()){
        update_target_current();

        if(printPosition) {
            LOG("motor %d position: %.2f", 0, SChassisIF::motor[0].motor_present_position);
            LOG("motor %d position: %.2f", 1, SChassisIF::motor[1].motor_present_position);
            LOG("target position: %.2f", SChassisIF::target_position);
        }

        if(printCurrent) {
            LOG("motor %d target_current: %d", 0, SChassisIF::motor[0].target_current);
            LOG("motor %d target_current: %d", 1, SChassisIF::motor[1].target_current);
        }

        if(printVelocity) {
            LOG("motor %d motor_present_velocity: %.2f", 0, SChassisIF::motor[0].motor_present_velocity);
            LOG("motor %d motor_present_velocity: %.2f", 1, SChassisIF::motor[1].motor_present_velocity);
            LOG("target velocity: %.2f", SChassisIF::target_velocity);
        }

        if(printPower) {
            LOG("chassis power: %.2f", Referee::power_heat_data.chassis_power);
            LOG("chassis power_buffer: %u", Referee::power_heat_data.chassis_power_buffer);
            LOG("chassis current: %u", Referee::power_heat_data.chassis_current);
            LOG("chassis voltage: %u", Referee::power_heat_data.chassis_volt);
        }

        SChassisIF::send_currents();
        sleep(TIME_MS2I(100));
    }
}

/**
 * @brief enable the chassis
 */
static void cmd_chassis_enable(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_enable left(0/1) right(0/1)");
        return;
    }
    if ((*argv[0] - '0') || (*argv[1] - '0')){
        SentryChassisSKD::turn_on();
    } else{
        SentryChassisSKD::turn_off();
    }
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

    SentryChassisSKD::set_maximum_velocity(Shell::atof(argv[0]));
}

/**
 * @brief set the mode for the sentry
 * @attention test_mode is true for testing, and is false for automatically driving
 */
static void cmd_chassis_set_mode(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "c_set_mode stop_mode(0)/one_step_mode(1)/shuttled_mode(2)/v_mode(3)/final_auto_mode(4)");
        return;
    }
    switch (*argv[0]){
        case ('1'):
            SentryChassisSKD::set_mode(SentryChassisSKD::ONE_STEP_MODE);
            break;
        case ('2'):
            SentryChassisSKD::set_mode(SentryChassisSKD::SHUTTLED_MODE);
            break;
        case ('3'):
            SentryChassisSKD::set_mode(SentryChassisSKD::V_MODE);
            break;
        case ('4'):
            SentryChassisSKD::set_mode(SentryChassisSKD::FINAL_AUTO_MODE);
            break;
//        case ('5'):
//            SentryChassisSKD::set_mode(SentryChassisSKD::POM_MODE);
//            break;
        default:
            SentryChassisSKD::set_mode(SentryChassisSKD::STOP_MODE);
    }
}

/**
 * @brief set chassis common PID params
 * @note modified to set POM pid !!
 */
static void cmd_chassis_set_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 7) {
        // shellUsage(chp, "g_set_params bullet(0)/plate(1) NULL(0)/v_to_i(1) ki kp kd i_limit out_limit");
        shellUsage(chp, "g_set_params 0or1whatever p2v(0)/v2i(1)/a2v(2) ki kp kd i_limit out_limit");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    PIDControllerBase::pid_params_t pid_params = {Shell::atof(argv[2]),
                                                  Shell::atof(argv[3]),
                                                  Shell::atof(argv[4]),
                                                  Shell::atof(argv[5]),
                                                  Shell::atof(argv[6])};

    // SentryChassisSKD::set_pid(*argv[1] == '0', pid_params);
    if (*argv[1] == '0')    // a2v -> p2v
        SentryChassisSKD::set_pid(2, pid_params);
    if (*argv[1] == '1')    // v2i
        SentryChassisSKD::set_pid(0, pid_params);
    if (*argv[1] == '2')    // a2v
        SentryChassisSKD::set_pid(1, pid_params);

    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

/**
 * @brief print the pid information of the specific motor
 * @note modified to print POM pid !!
 */
static void cmd_chassis_print_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_echo_params");
        return;
    }
//    chprintf(chp, "bullet v_to_i:       ");
//    SentryChassisSKD::print_pid(true);
//    chprintf(chp, "plate v_to_i:     ");
//    SentryChassisSKD::print_pid(false);
    chprintf(chp, "P to V:     ");
    SentryChassisSKD::print_pid(2);
    chprintf(chp, "A to V:     ");
    SentryChassisSKD::print_pid(1);
    chprintf(chp, "V to I:     ");
    SentryChassisSKD::print_pid(0);

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
    SentryChassisSKD::set_destination(Shell::atof(argv[0]));
}

/**
 * @brief set the poser limit for POM mode, Watts
 * @note need to set manually in shell
 */
 static void cmd_chassis_set_power_limit(BaseSequentialStream *chp, int argc, char **argv){
    (void) argv;
    if (argc != 1){
        shellUsage(chp, "c_set_p");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    SentryChassisSKD::set_target_power(Shell::atof(argv[0]));
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
    SentryChassisSKD::set_origin();
}

static void cmd_chassis_print_position(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0){
        shellUsage(chp, "c_pos");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    SentryChassisSKD::printPosition = !SentryChassisSKD::printPosition;
}

static void cmd_chassis_print_current(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0){
        shellUsage(chp, "c_cur");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    SentryChassisSKD::printCurrent = !SentryChassisSKD::printCurrent;
}

static void cmd_chassis_print_velocity(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0){
        shellUsage(chp, "c_v");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    SentryChassisSKD::printVelocity = !SentryChassisSKD::printVelocity;
}

static void cmd_chassis_print_power(BaseSequentialStream *chp, int argc, char *argv[]){
    (void) argv;
    if (argc != 0){
        shellUsage(chp, "c_p");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }
    SentryChassisSKD::printPower = !SentryChassisSKD::printPower;
}

// Shell commands to control the chassis
ShellCommand chassisCommands[] = {
        {"g_enable",   cmd_chassis_enable},
        {"g_enable_fb", cmd_chassis_enable_feedback},
        {"g_set_v",     cmd_set_target_velocities},
        {"c_set_mode",    cmd_chassis_set_mode},
        {"g_set_params",  cmd_chassis_set_pid},         // modified to set POM pid
        {"g_echo_params",   cmd_chassis_print_pid},     // modified to echo POM pid
        {"g_set_angle",   cmd_chassis_set_position},
        {"c_set_p", cmd_chassis_set_power_limit},       // added to set power limit, default is 20 when init()
        {"g_fix", cmd_chassis_clear_position},
        {"c_pos", cmd_chassis_print_position},
        {"c_cur", cmd_chassis_print_current},
        {"c_v", cmd_chassis_print_velocity},
        {"c_p", cmd_chassis_print_power},
        {nullptr,    nullptr}
};

CANInterface can1(&CAND1);

int main(void) {
    halInit();
    System::init();
    LED::green_off();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(chassisCommands);

    can1.start(HIGHPRIO - 1);
    Referee::init();
    SentryChassisIF::init(&can1);
    SentryChassisSKD::sentryChassisThread.start(NORMALPRIO);
    chassisFeedbackThread.start(NORMALPRIO - 1);

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