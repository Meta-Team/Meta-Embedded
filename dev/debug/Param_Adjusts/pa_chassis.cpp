//
// Created by liuzikai on 2019-02-03.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "chassis_interface.h"
#include "pid_controller.hpp"


#include "vehicle/infantry/vehicle_infantry.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

// PID Related
float target_velocity[4];
int target_current[4];
PIDController v2i_pid[4];

int install_mode_ = 1;
float v_to_wheel_angular_velocity_ = 0.0f;
float w_to_v_ratio_ = 0.0f;
float chassis_gimbal_offset_ = 0.0f;

float target_vx = 0.0f; // [mm/s]
float target_vy = 0.0f; // [mm/s]
float target_w = 0.0f;  // [degree/s], negative value for clockwise)

time_msecs_t test_end_time = 0; // [ms]

unsigned const CHASSIS_FEEDBACK_INTERVAL = 25; // [ms]
unsigned const CHASSIS_THREAD_INTERVAL = 2;    // [ms]

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

static void velocity_decompose_(float vx, float vy, float w) {

    // FR, +vx, -vy, +w
    // FL, +vx, +vy, +w, since the motor is installed in the opposite direction
    // BL, -vx, +vy, +w, since the motor is installed in the opposite direction
    // BR, -vx, -vy, +w

    target_velocity[ChassisIF::FR] = install_mode_ * (+vx - vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[ChassisIF::FR] = (int) v2i_pid[ChassisIF::FR].calc(ChassisIF::feedback[ChassisIF::FR]->actual_velocity, target_velocity[ChassisIF::FR]);

    target_velocity[ChassisIF::FL] = install_mode_ * (+vx + vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[ChassisIF::FL] = (int) v2i_pid[ChassisIF::FL].calc(ChassisIF::feedback[ChassisIF::FL]->actual_velocity, target_velocity[ChassisIF::FL]);

    target_velocity[ChassisIF::BL] = install_mode_ * (-vx + vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[ChassisIF::BL] = (int) v2i_pid[ChassisIF::BL].calc(ChassisIF::feedback[ChassisIF::BL]->actual_velocity, target_velocity[ChassisIF::BL]);

    target_velocity[ChassisIF::BR] = install_mode_ * (-vx - vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[ChassisIF::BR] = (int) v2i_pid[ChassisIF::BR].calc(ChassisIF::feedback[ChassisIF::BR]->actual_velocity, target_velocity[ChassisIF::BR]);
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
    for (int i = 0; i < 4; i++) {
        v2i_pid[i].change_parameters({Shell::atof(argv[0]),
                                         Shell::atof(argv[1]),
                                         Shell::atof(argv[2]),
                                         Shell::atof(argv[3]),
                                         Shell::atof(argv[4])});
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
    PIDController::pid_params_t p = v2i_pid[0].get_parameters(); // all PID params should be the same
    chprintf(chp, "Chassis PID: %f %f %f %f %f" SHELL_NEWLINE_STR, p.kp, p.ki, p.kd, p.i_limit, p.out_limit);
}

// Shell commands to control the chassis
ShellCommand chassisCommands[] = {
        {"c_set_params",  cmd_chassis_set_parameters},
        {"c_set_target",  cmd_chassis_set_target},
        {"c_echo_params", cmd_chassis_echo_parameters},
        {nullptr,         nullptr}
};

class ChassisFeedbackThread : public BaseStaticThread<1024> {
private:
    void main() final {
        setName("chassis");
        while (!shouldTerminate()) {
            Shell::printf("!cv,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f" SHELL_NEWLINE_STR,
                          TIME_I2MS(chibios_rt::System::getTime()),
                          ChassisIF::feedback[0]->actual_velocity,
                          target_velocity[0],
                          ChassisIF::feedback[1]->actual_velocity,
                          target_velocity[1],
                          ChassisIF::feedback[2]->actual_velocity,
                          target_velocity[2],
                          ChassisIF::feedback[3]->actual_velocity,
                          target_velocity[3]);
            sleep(TIME_MS2I(CHASSIS_FEEDBACK_INTERVAL));
        }
    }
} chassisFeedbackThread;

class ChassisPIDThread : public BaseStaticThread<512> {
private:
    unsigned int timeTrig = SYSTIME+1000;
    bool ison = false;
    void main() final {
        setName("chassisPID");
        while(!shouldTerminate()) {
            // An LED that indicate the system status. (flash when the program is not mad)
            if(SYSTIME > timeTrig) {
                if(ison) {
                    LED::led_off(2);
                } else {
                    LED::led_on(2);
                }
                timeTrig+=1000;
                ison = !ison;
            }
            if(SYSTIME < test_end_time) {
                velocity_decompose_(target_vx, target_vy, target_w);
                for (int i = 0; i < 4; i++ ){
                    *ChassisIF::target_current[i] = target_current[i];
                }
            } else {
                for (int i = 0; i < 4; i++ ) {
                    target_velocity[i] = 0.0f;
                    *ChassisIF::target_current[i] = 0;
                }
            }
            sleep(TIME_MS2I(CHASSIS_FEEDBACK_INTERVAL));
        }
    }
} chassisPidThread;

int main(void) {
    
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO-4);
    Shell::addCommands(chassisCommands);

    // Data Configuration
    v_to_wheel_angular_velocity_ = (360.0f / CHASSIS_WHEEL_CIRCUMFERENCE);
    chassis_gimbal_offset_ = 0;
    install_mode_ = 1;
    w_to_v_ratio_ = (CHASSIS_WHEEL_BASE + CHASSIS_WHEEL_TREAD) / 2.0f / 360.0f * 3.14159f;

    can1.start(HIGHPRIO-2, HIGHPRIO-3);
    can2.start(HIGHPRIO, HIGHPRIO - 1);

    chassisFeedbackThread.start(NORMALPRIO - 1);
    ChassisIF::motor_can_config_t CHASSIS_MOTOR_CONFIG_[ChassisIF::MOTOR_COUNT] = CHASSIS_MOTOR_CONFIG;
    ChassisIF::init(&can1, &can2, CHASSIS_MOTOR_CONFIG_);

    LED::led_on(1);
    chassisPidThread.start(NORMALPRIO);

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
