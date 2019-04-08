//
// Created by liuzikai on 2019-01-27.
//

// Basic headers (including board definitions, so they should be at the very beginning)
#include "ch.hpp"
#include "hal.h"

#if defined(INFANTRY_ONE) // defined in CMakeLists.txt.

#include "vehicle_infantry_one.h"

#else
#error "main_infantry.cpp should only be used for Infantry #1."
#endif

#if defined(BOARD_RM_2017) // defined in board.h (included in hal.h)
#define STARTUP_BUTTON_PAD GPIOD
#define STARTUP_BUTTON_PIN_ID GPIOD_USER_BUTTON
#define STARTUP_BUTTON_PRESS_PAL_STATUS PAL_LOW
#elif defined(BOARD_RM_2018_A) // defined in board.h (included in hal.h)
#define STARTUP_BUTTON_PAD GPIOB
#define STARTUP_BUTTON_PIN_ID GPIOB_USER_BUTTON
#define STARTUP_BUTTON_PRESS_PAL_STATUS PAL_HIGH
#else
#error "Infantry #1 is only developed for RM board 2017 and RM board 2018 A."
#endif

// Debug headers
#include "led.h"
#include "serial_shell.h"

// Modules and basic communication channels
#include "can_interface.h"
#include "chassis_common.h"
#include "common_macro.h"

// Interfaces
#include "buzzer.h"
#include "mpu6500.h"
#include "remote_interpreter.h"
#include "gimbal_interface.h"
#include "chassis_interface.h"

// Controllers
#include "gimbal_calculator.h"
#include "chassis_calculator.h"

/**
 * Mode Table:
 * ------------------------------------------------------------
 * Left  Right  Mode
 * ------------------------------------------------------------
 *  UP    *     Safe
 *  MID   UP    Remote - Gimbal (right) + Chassis XY (left)
 *  MID   MID   Remote - Gimbal (right) + Shooting (left)
 *  MID   DOWN  Remote - Chassis XY (right) + Chassis W (left)
 *  DOWN  *     PC - Gimbal + Chassis + Shooting
 *  -Others-    Safe
 * ------------------------------------------------------------
 */

/**
 * PC Control:
 * WSAD move, QE rotate.
 * Move mouse to move gimbal. Press left to shoot 6 bullet. Press right to reset gimbal to front direction.
 */

/** Declarations **/

CANInterface can1(&CAND1);

/** Threads **/

/**
 * @name GimbalThread
 * @brief Thread to control gimbal, including shooting mechanism
 * @pre Remote interpreter starts receive
 * @pre Initialize GimbalInterface with CAN driver and set the front angles of yaw and pitch properly
 * @pre Start the thread of updating MPU6500
 */
class GimbalThread : public chibios_rt::BaseStaticThread<1024> {

    static constexpr unsigned int GIMBAL_THREAD_INTERVAL = 10; // [ms]

    static constexpr float REMOTE_FRICTION_WHEEL_DUTY_CYCLE = 1.0;

    static constexpr float PC_YAW_SPEED_RATIO = 54000; // rotation speed when mouse moves at the fastest limit [degree/s]
    static constexpr float PC_YAW_ANGLE_LIMITATION = 60; // maximum range (both CW and CCW) [degree]

    static constexpr float PC_PITCH_SPEED_RATIO = 12000; // rotation speed when mouse moves at the fastest limit [degree/s]
    static constexpr float PC_PITCH_ANGLE_LIMITATION = 15; // maximum range (both up and down) [degree]

    float pc_yaw_current_target_angle = 0;
    float pc_pitch_current_target_angle = 0;

    bool pc_right_pressed = false;

    static constexpr float PC_FRICTION_WHEEL_DUTY_CYCLE = 1.0;

    void main() final {
        setName("gimbal");

        /*** Parameters Set up***/
        GimbalController::yaw.v_to_i_pid.change_parameters(GIMBAL_PID_YAW_V2I_PARAMS);
        GimbalController::yaw.angle_to_v_pid.change_parameters(GIMBAL_PID_YAW_A2V_PARAMS);
        GimbalController::pitch.v_to_i_pid.change_parameters(GIMBAL_PID_PITCH_V2I_PARAMS);
        GimbalController::pitch.angle_to_v_pid.change_parameters(GIMBAL_PID_PITCH_A2V_PARAMS);
        GimbalInterface::yaw.enabled = GimbalInterface::pitch.enabled = true;

        GimbalController::bullet_loader.v_to_i_pid.change_parameters(GIMBAL_PID_BULLET_LOADER_V2I_PARAMS);
        GimbalInterface::bullet_loader.enabled = GimbalInterface::friction_wheels.enabled = true;

        while (!shouldTerminate()) {

            /*** Yaw and Pitch Motors ***/
            if (Remote::rc.s1 == Remote::RC_S_MIDDLE &&
                (Remote::rc.s2 == Remote::RC_S_UP || Remote::rc.s2 == Remote::RC_S_MIDDLE)) {
                // Remote control mode

                // PID #1: target angle -> target velocity
                float yaw_target_velocity = GimbalController::yaw.angle_to_v(GimbalInterface::yaw.actual_angle,
                                                                             -Remote::rc.ch0 * 60);
                float pitch_target_velocity = GimbalController::pitch.angle_to_v(GimbalInterface::pitch.actual_angle,
                                                                                 Remote::rc.ch1 * 20);
                // PID #2: target velocity -> target current
                GimbalInterface::yaw.target_current = (int) GimbalController::yaw.v_to_i(
                        GIMBAL_YAW_ACTUAL_VELOCITY, yaw_target_velocity);
                GimbalInterface::pitch.target_current = (int) GimbalController::pitch.v_to_i(
                        GIMBAL_PITCH_ACTUAL_VELOCITY, pitch_target_velocity);

            } else if (Remote::rc.s1 == Remote::RC_S_DOWN) { // PC control mode

                // If V is pressed, reset target angle
                if (Remote::key.v) {
                    pc_yaw_current_target_angle = pc_pitch_current_target_angle = 0;
                } else {
                    pc_yaw_current_target_angle +=
                            -Remote::mouse.x * (PC_YAW_SPEED_RATIO * (GIMBAL_THREAD_INTERVAL / 1000.0f));
                    pc_pitch_current_target_angle +=
                            -Remote::mouse.y * (PC_PITCH_SPEED_RATIO * (GIMBAL_THREAD_INTERVAL / 1000.0f));
                }

                ABS_LIMIT(pc_yaw_current_target_angle, PC_YAW_ANGLE_LIMITATION);
                ABS_LIMIT(pc_pitch_current_target_angle, PC_PITCH_ANGLE_LIMITATION);

                // TODO: find a better way to handle the following duplicate code
                // PID #1: target angle -> target velocity
                float yaw_target_velocity = GimbalController::yaw.angle_to_v(GimbalInterface::yaw.actual_angle,
                                                                             pc_yaw_current_target_angle);
                float pitch_target_velocity = GimbalController::pitch.angle_to_v(GimbalInterface::pitch.actual_angle,
                                                                                 pc_pitch_current_target_angle);
                // PID #2: target velocity -> target current
                GimbalInterface::yaw.target_current = (int) GimbalController::yaw.v_to_i(
                        GIMBAL_YAW_ACTUAL_VELOCITY, yaw_target_velocity);
                GimbalInterface::pitch.target_current = (int) GimbalController::pitch.v_to_i(
                        GIMBAL_PITCH_ACTUAL_VELOCITY, pitch_target_velocity);

            } else {
                GimbalInterface::yaw.target_current = GimbalInterface::pitch.target_current = 0;
            }

            /*** Friction Wheels and Bullet Loader ***/
            GimbalController::bullet_loader.update_accumulation_angle(
                    GimbalInterface::bullet_loader.get_accumulate_angle());
            if (Remote::rc.s1 == Remote::RC_S_MIDDLE && Remote::rc.s2 == Remote::RC_S_MIDDLE) {
                GimbalInterface::friction_wheels.duty_cycle = REMOTE_FRICTION_WHEEL_DUTY_CYCLE;
                if (Remote::rc.ch3 < 0.1) {
                    if (!GimbalController::bullet_loader.get_shooting_status()) {
                        GimbalController::bullet_loader.start_continuous_shooting();
                    }
                    GimbalInterface::bullet_loader.target_current = (int) GimbalController::bullet_loader.get_target_current(
                            GimbalInterface::bullet_loader.angular_velocity, Remote::rc.ch3 * 360);
                } else {
                    if (GimbalController::bullet_loader.get_shooting_status()) {
                        GimbalController::bullet_loader.stop_shooting();
                    }
                }
            } else if (Remote::rc.s1 == Remote::RC_S_DOWN) { // PC control mode

                // If mouse left button is pressed, start continuous shooting
                if (Remote::mouse.press_left) {
                    // TODO: enable friction in emergency
                    if (!GimbalController::bullet_loader.get_shooting_status() &&
                        GimbalInterface::friction_wheels.duty_cycle > 0) {
                        GimbalController::bullet_loader.start_continuous_shooting();
                    }

                } else {

                    if (GimbalController::bullet_loader.get_shooting_status()) {
                        GimbalController::bullet_loader.stop_shooting();
                    }
                }

                if (Remote::mouse.press_right) {
                    if (!pc_right_pressed) {
                        if (GimbalInterface::friction_wheels.duty_cycle == 0) {
                            GimbalInterface::friction_wheels.duty_cycle = PC_FRICTION_WHEEL_DUTY_CYCLE;
                        } else {
                            GimbalInterface::friction_wheels.duty_cycle = 0;
                        }
                        pc_right_pressed = true;
                    }
                } else {
                    if (pc_right_pressed) {
                        pc_right_pressed = false;
                    }
                }

                GimbalInterface::bullet_loader.target_current = (int) GimbalController::bullet_loader.get_target_current(
                        GimbalInterface::bullet_loader.angular_velocity, -270);

            } else {
                GimbalInterface::friction_wheels.duty_cycle = 0;
                GimbalInterface::bullet_loader.target_current = 0;
            }

            GimbalInterface::send_gimbal_currents();

            sleep(TIME_MS2I(GIMBAL_THREAD_INTERVAL));
        }
    }
} gimbalThread;

/**
 * @name ChassisThread
 * @brief thread to control chassis
 * @pre RemoteInterpreter start receive
 * @pre initialize ChassisInterface with CAN driver
 */
class ChassisThread : public chibios_rt::BaseStaticThread<1024> {

    static constexpr unsigned int chassis_thread_interval = 20;

    static constexpr float PC_W_VY = -600.0f;
    static constexpr float PC_S_VY = 600.0f;
    static constexpr float PC_E_VX = -600.0f;
    static constexpr float PC_Q_VX = 600.0f;
    static constexpr float PC_A_W = -150.0f;
    static constexpr float PC_D_W = 150.0f;

    static constexpr float PC_CTRL_RATIO = 0.5f;

    void main() final {

        setName("chassis");

        ChassisController::change_pid_params(CHASSIS_PID_V2I_PARAMS);

        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::RC_S_MIDDLE &&
                (Remote::rc.s2 == Remote::RC_S_UP || Remote::rc.s2 == Remote::RC_S_DOWN)) {

                float target_vx = -Remote::rc.ch2 * 1000.0f;
                float target_vy = -Remote::rc.ch3 * 1000.0f;
                float target_w;
                if (Remote::rc.s2 == Remote::RC_S_DOWN) {
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

            } else if (Remote::rc.s1 == Remote::RC_S_DOWN) { // PC control mode

                float target_vx, target_vy, target_w;

                if (Remote::key.w) target_vy = PC_W_VY;
                else if (Remote::key.s) target_vy = PC_S_VY;
                else target_vy = 0;

                if (Remote::key.q) target_vx = PC_Q_VX;
                else if (Remote::key.e) target_vx = PC_E_VX;
                else target_vx = 0;

                if (Remote::key.a) target_w = PC_A_W;
                else if (Remote::key.d) target_w = PC_D_W;
                else target_w = 0;

                if (Remote::key.ctrl) {
                    target_vx *= PC_CTRL_RATIO;
                    target_vy *= PC_CTRL_RATIO;
                    target_w *= PC_CTRL_RATIO;
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

            } else {
                for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                    ChassisInterface::motor[i].target_current = 0;
                }
            }

            ChassisInterface::send_chassis_currents();
            sleep(TIME_MS2I(chassis_thread_interval));

        }
    }
} chassisThread;


int main(void) {

    /*** --------------------------- Period 1. Basic Setup --------------------------- ***/

    /** Basic Initializations **/
    halInit();
    chibios_rt::System::init();

    LED::green_off();
    LED::red_off();

    /** Debug Setup **/
    Shell::start(HIGHPRIO);

    /** Basic IO Setup **/
    can1.start(HIGHPRIO - 1);
    MPU6500Controller::start(HIGHPRIO - 2);
    Remote::start_receive();

    GimbalInterface::init(&can1, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);
    ChassisInterface::init(&can1);


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

//    while (palReadPad(STARTUP_BUTTON_PAD, STARTUP_BUTTON_PIN_ID) != STARTUP_BUTTON_PRESS_PAL_STATUS) {
//        // Wait for the button to be pressed
//        LED::green_toggle();
//        chThdSleepMilliseconds(300);
//    }
//    /** User has pressed the button **/

    LED::green_on();

//    /** Gimbal Calibration **/
//    GimbalInterface::yaw.reset_front_angle();
//    GimbalInterface::pitch.reset_front_angle();

    /** Echo Gimbal Raws and Converted Angles **/
    chThdSleepMilliseconds(500);
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        GimbalInterface::yaw.last_angle_raw, GimbalInterface::yaw.actual_angle,
        GimbalInterface::pitch.last_angle_raw, GimbalInterface::pitch.actual_angle);

    /** Start Logic Control Thread **/
    gimbalThread.start(NORMALPRIO);
    chassisThread.start(NORMALPRIO - 1);

    /** Play the Startup Sound **/
    Buzzer::play_sound(Buzzer::sound_startup_intel, LOWPRIO);


    /*** ------------------------ Period 3. End of main thread ----------------------- ***/

    // Entering empty loop with low priority

#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(1);
#endif
    return 0;
}