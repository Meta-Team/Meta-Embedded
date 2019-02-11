//
// Created by liuzikai on 2019-02-03.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "chassis_interface.h"
#include "chassis_controller.h"

using namespace chibios_rt;

/**
 * @brief callback function for CAN1
 * @param rxmsg
 */
static void can1_callback(CANRxFrame *rxmsg) {
    switch (rxmsg->SID) {
        case 0x201:
        case 0x202:
        case 0x203:
        case 0x204:
            ChassisInterface::process_chassis_feedback(rxmsg);
            break;
        default:
            break;
    }
}

CANInterface can1(&CAND1, can1_callback);

float target_vx = 0.0f; // (mm/s)
float target_vy = 0.0f; // (mm/s)
float target_w = 0.0f; // (degree/s, negative value for clockwise)

time_msecs_t test_end_time = 0; // [ms]

int const chassis_feedback_interval = 25; // ms
int const chassis_thread_interval = 20; // ms

/**
 * @brief set chassis controller target
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_set_target(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 4) {
        shellUsage(chp, "c_set_target vx(mm/s) vy(mm/s) w(deg/s, + for ccw) test_time(ms)");
        return;
    }

    target_vx = Shell::atof(argv[0]);
    target_vy = Shell::atof(argv[1]);
    target_w = Shell::atof(argv[2]);

    test_end_time = TIME_I2MS(chVTGetSystemTime()) + (time_msecs_t) Shell::atoi(argv[3]);

}

/**
 * @brief set chassis common PID params
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "c_set_params ki kp kd i_limit out_limit");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);  // echo chassis parameters error
        return;
    }


    ChassisController::change_pid_params(Shell::atof(argv[0]),
                                         Shell::atof(argv[1]),
                                         Shell::atof(argv[2]),
                                         Shell::atof(argv[3]),
                                         Shell::atof(argv[4]));
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        ChassisController::motor[i].pid.clear_i_out();
    }
    chprintf(chp, "!cps" SHELL_NEWLINE_STR); // echo chassis parameters set
}

/**
 * @brief echo chassis PID parameters
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_chassis_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_echo_params");
        return;
    }
    chprintf(chp, "Chassis PID: %f %f %f %f %f" SHELL_NEWLINE_STR,
             ChassisController::motor[0].pid.kp,
             ChassisController::motor[0].pid.ki,
             ChassisController::motor[0].pid.kd,
             ChassisController::motor[0].pid.i_limit,
             ChassisController::motor[0].pid.out_limit); // echo chassis parameters
}

// Shell commands to control the chassis
ShellCommand chassisCommands[] = {
        {"c_set_params", cmd_chassis_set_parameters},
        {"c_set_target", cmd_chassis_set_target},
        {"c_echo_params", cmd_chassis_echo_parameters},
        {nullptr,        nullptr}
};

class ChassisFeedbackThread : public BaseStaticThread<512> {
private:
    void main() final {
        setName("chassis");
        while (!shouldTerminate()) {
            Shell::printf("!cv,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f" SHELL_NEWLINE_STR,
                          TIME_I2MS(chibios_rt::System::getTime()),
                          ChassisInterface::motor[0].actual_angular_velocity,
                          ChassisController::motor[0].target_velocity,
                          ChassisInterface::motor[1].actual_angular_velocity,
                          ChassisController::motor[1].target_velocity,
                          ChassisInterface::motor[2].actual_angular_velocity,
                          ChassisController::motor[2].target_velocity,
                          ChassisInterface::motor[3].actual_angular_velocity,
                          ChassisController::motor[3].target_velocity);
            sleep(TIME_MS2I(chassis_feedback_interval));
        }
    }
} chassisFeedbackThread;

class ChassisThread : public BaseStaticThread<512> {
protected:
    void main() final {
        setName("chassis");
        while (!shouldTerminate()) {

            if (target_vx != 0.0f || target_vy != 0.0f || target_w != 0.0f) {

                if (TIME_I2MS(chVTGetSystemTime()) >= test_end_time) {

                    target_vx = target_vy = target_w = 0.0f;

                    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                        ChassisInterface::motor[i].target_current = 0;
                    }

                    Shell::printf("!ce" SHELL_NEWLINE_STR);

                } else {

                    // Pack the actual velocity into an array
                    float measured_velocity[4];
                    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                        measured_velocity[i] = ChassisInterface::motor[i].actual_angular_velocity;
                    }

                    // Perform calculation
                    ChassisController::calc(measured_velocity, target_vx, target_vy, target_w);

                    // Pass the target current to interface
                    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                        ChassisInterface::motor[i].target_current = (int) ChassisController::motor[i].target_current;
                    }
                }
            }

            ChassisInterface::send_chassis_currents();

            sleep(TIME_MS2I(chassis_thread_interval));
        }
    }
} chassisThread;


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(chassisCommands);


//    can1.start_can();
    can1.start_thread(HIGHPRIO - 1);
    ChassisInterface::set_can_interface(&can1);

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
