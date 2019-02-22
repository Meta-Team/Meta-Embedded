//
// Created by liuzikai on 2019-01-27.
//

// Header for vehicle. VEHICLE is set for each target in CMakeLists.txt.

#define INFANTRY_ONE 1
#define HERO 2
#define ENGINEER 3
#define INFANTRY_TWO 4
#define INFANTRY_THREE 5

#if VEHICLE == INFANTRY_ONE

#include "vehicle_infantry_one.h"

#elif VEHICLE == ENGINEER
#include "vehicle_engineer.h"
#endif

//#if defined(BOARD_RM_2017) // press to be low
#define STARTUP_BUTTON_PRESS_PAL_STATUS PAL_LOW
//#elif defined(BOARD_RM_2018_A) // press to be high
//#define STARTUP_BUTTON_PRESS_PAL_STATUS PAL_HIGH
//#endif

// Basic headers
#include "ch.hpp"
#include "hal.h"

// Debug headers
#include "led.h"
#include "serial_shell.h"

// Modules and basic communication channels
#include "can_interface.h"
#include "chassis_common.h"

// Interfaces
#include "buzzer.h"
#include "mpu6500.h"
#include "remote_interpreter.h"
#include "gimbal_interface.h"
#include "chassis_interface.h"

// Controllers
#include "gimbal_controller.h"
#include "chassis_controller.h"

/**
 * Mode Table:
 * ------------------------------------------------------------
 * Left  Right  Mode
 * ------------------------------------------------------------
 *  UP    *     Safe
 *  MID   UP    Remote - Gimbal (right) + Chassis XY (left)
 *  MID   MID   Remote - Gimbal (right) + Shooting (left)
 *  MID   DOWN  Remote - Chassis XY (right) + Chassis W (left)
 *  -Others-    Safe
 * ------------------------------------------------------------
 */


/** Declarations **/

CANInterface can1(&CAND1);

/** Threads **/

/**
 * @brief thread to update MPU6500 data
 */
class MPU6500Thread : public chibios_rt::BaseStaticThread<512> {
    static constexpr unsigned int update_interval = 25; // [ms]

    void main() final {
        setName("mpu6500");
        MPU6500Controller::start();
        while (!shouldTerminate()) {
            MPU6500Controller::getData();
            sleep(TIME_MS2I(update_interval));
        }
    }
} mpu6500Thread;

/**
 * @brief thread to control gimbal, including shooting mechanism
 * @pre RemoteInterpreter start receive
 * @pre initialize GimbalInterface with CAN driver
 * @pre start the thread of updating MPU6500
 * @pre reset front angle properly
 */
class GimbalThread : public chibios_rt::BaseStaticThread<1024> {

    static constexpr unsigned int gimbal_thread_interval = 10; // [ms]
    static constexpr float gimbal_remote_mode_friction_wheel_duty_cycle = 1.0;

    void main() final {
        setName("gimbal");

        GimbalController::yaw.v_to_i_pid.change_parameters(GIMBAL_PID_YAW_V2I_PARAMS);
        GimbalController::yaw.angle_to_v_pid.change_parameters(GIMBAL_PID_YAW_A2V_PARAMS);
        GimbalController::pitch.v_to_i_pid.change_parameters(GIMBAL_PID_PITCH_V2I_PARAMS);
        GimbalController::pitch.angle_to_v_pid.change_parameters(GIMBAL_PID_PITCH_A2V_PARAMS);

        GimbalInterface::yaw.enabled = GimbalInterface::pitch.enabled = true;

        GimbalController::bullet_loader.v_to_i_pid.change_parameters(GIMBAL_PID_BULLET_LOADER_V2I_PARAMS);
        GimbalInterface::bullet_loader.enabled = GimbalInterface::friction_wheels.enabled = true;

        bool has_start_incontinuous_mode = false;

        while (!shouldTerminate()) {

            /*** Yaw and Pitch Motors ***/
            if (Remote::rc.s1 == Remote::REMOTE_RC_S_MIDDLE &&
                (Remote::rc.s2 == Remote::REMOTE_RC_S_UP || Remote::rc.s2 == Remote::REMOTE_RC_S_MIDDLE)) {

                // Target angle -> target velocity
                float yaw_target_velocity = GimbalController::yaw.angle_to_v(GimbalInterface::yaw.actual_angle,
                                                                             -Remote::rc.ch0 * 60);
                float pitch_target_velocity = GimbalController::pitch.angle_to_v(GimbalInterface::pitch.actual_angle,
                                                                                 -Remote::rc.ch1 * 20);
                // Target velocity -> target current
                GimbalInterface::yaw.target_current = (int) GimbalController::yaw.v_to_i(
                        GIMBAL_YAW_ACTUAL_VELOCITY, yaw_target_velocity);
                GimbalInterface::pitch.target_current = (int) GimbalController::pitch.v_to_i(
                        GIMBAL_PITCH_ACTUAL_VELOCITY, pitch_target_velocity);

            } else {
                GimbalInterface::yaw.target_current = GimbalInterface::pitch.target_current = 0;
            }

            /*** Friction Wheels and Bullet Loader ***/
            if (Remote::rc.s1 == Remote::REMOTE_RC_S_MIDDLE && Remote::rc.s2 == Remote::REMOTE_RC_S_MIDDLE) {
                GimbalInterface::friction_wheels.duty_cycle = gimbal_remote_mode_friction_wheel_duty_cycle;
                GimbalController::bullet_loader.update_accumulation_angle(GimbalInterface::bullet_loader.get_accumulate_angle());
                if (Remote::rc.ch3 < 0.1) {
                    if (!GimbalController::bullet_loader.get_shooting_status()) {
                        GimbalController::bullet_loader.start_continuous_shooting();
                    }
                    GimbalInterface::bullet_loader.target_current = (int) GimbalController::bullet_loader.get_target_current(
                            GimbalInterface::bullet_loader.angular_velocity, -Remote::rc.ch3 * 360);
                } else if (Remote::rc.ch3 > 0.2) {
                    if (!GimbalController::bullet_loader.get_shooting_status() && !has_start_incontinuous_mode) {
                        GimbalController::bullet_loader.start_incontinuous_shooting(10);
                        has_start_incontinuous_mode = true;
                    }
                    GimbalInterface::bullet_loader.target_current = (int) GimbalController::bullet_loader.get_target_current(
                            GimbalInterface::bullet_loader.angular_velocity, 270);
                } else {
                    if (GimbalController::bullet_loader.get_shooting_status()) {
                        GimbalController::bullet_loader.stop_shooting();
                        has_start_incontinuous_mode = false;
                    }
                }
            } else {
                GimbalInterface::friction_wheels.duty_cycle = 0;
                GimbalInterface::bullet_loader.target_current = 0;
            }

            GimbalInterface::send_gimbal_currents();

            sleep(TIME_MS2I(gimbal_thread_interval));
        }
    }
} gimbalThread;

// TODO: add comments
class ChassisThread : public chibios_rt::BaseStaticThread<1024> {
    static constexpr unsigned int chassis_thread_interval = 20;

    void main() final {

        setName("chassis");

        ChassisController::change_pid_params(CHASSIS_PID_V2I_PARAMS);

        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::REMOTE_RC_S_MIDDLE &&
                (Remote::rc.s2 == Remote::REMOTE_RC_S_UP || Remote::rc.s2 == Remote::REMOTE_RC_S_DOWN)) {

                float target_vx = -Remote::rc.ch2 * 1000.0f;
                float target_vy = -Remote::rc.ch3 * 1000.0f;
                float target_w;
                if (Remote::rc.s2 == Remote::REMOTE_RC_S_DOWN) {
                    target_w = Remote::rc.ch0 * 180.0f;
                } else {
                    target_w = 0;
                }

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

            ChassisInterface::send_chassis_currents();
            sleep(TIME_MS2I(chassis_thread_interval));

        }
    }
} chassisThread;

int main(void) {

    // Basic initialization
    halInit();
    chibios_rt::System::init();
    LED::green_off();
    LED::red_off();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);

    can1.start(HIGHPRIO - 1);

    mpu6500Thread.start(HIGHPRIO - 2);

    Remote::start_receive();

    GimbalInterface::init(&can1);
    ChassisInterface::init(&can1);

    while (palReadPad(GPIOD, 10) != STARTUP_BUTTON_PRESS_PAL_STATUS) {
        // Wait for the button to be pressed
        LED::green_toggle();
        chThdSleepMilliseconds(300);
    }
    LED::green_on();

    GimbalInterface::yaw.reset_front_angle();
    GimbalInterface::pitch.reset_front_angle();

    gimbalThread.start(NORMALPRIO);
    chassisThread.start(NORMALPRIO - 1);

    Buzzer::play_sound(Buzzer::sound_startup_intel, LOWPRIO);

#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle_configs() quits, the vehicle_configs thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(1);
#endif
    return 0;
}