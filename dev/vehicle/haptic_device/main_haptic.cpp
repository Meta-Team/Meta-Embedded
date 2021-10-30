//
// Created by 钱晨 on 10/29/21.
//

/// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "common_macro.h"

#include "shell.h"
#include "interface/can/can_interface.h"
#include "ahrs.h"

#include "thread_priorities.h"

/// Vehicle Specific Configurations
#if defined(HAPTIC_DVC)
#include "AHRS_PARAMS.h"
#include "CANBUS_MOTOR_CFG.h"
#include "DEBUG_CHECK.h"
#include "can_motor_interface.h"
#else
#error "File main_infantry.cpp should only be used for Infantry #3, #4, #5."
#endif

/// Board Guard
#if defined(BOARD_RM_2018_A)
#else
#error "Infantry supports only RM Board 2018 A currently"
#endif

/// Instances
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard ahrs;

/// Local Constants
static const Matrix33 ON_BOARD_AHRS_MATRIX_ = ON_BOARD_AHRS_MATRIX;
static const Matrix33 GIMBAL_ANGLE_INSTALLATION_MATRIX_ = GIMBAL_ANGLE_INSTALLATION_MATRIX;
static const Matrix33 GIMBAL_GYRO_INSTALLATION_MATRIX_ = GIMBAL_GYRO_INSTALLATION_MATRIX;

int main() {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    // Enable power of bullet loader motor
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    // Enable power of ultraviolet lights
    palSetPadMode(GPIOH, GPIOH_POWER2_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER2_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    /// Preparation of Period 1
    LED::all_off();

    /// Setup Shell
    Shell::start(THREAD_SHELL_PRIO);
    chThdSleepMilliseconds(50);  // wait for logo to print :)

    BuzzerSKD::init(THREAD_BUZZER_SKD_PRIO);

    /// Setup SDCard

    LED::led_on(DEV_BOARD_LED_SYSTEM_INIT);  // LED 1 on now

    /// Setup CAN1 & CAN2
    can1.start(THREAD_CAN1_RX_PRIO);
    can2.start(THREAD_CAN2_RX_PRIO);
    chThdSleepMilliseconds(5);
    //Inspector::startup_check_can();  // check no persistent CAN Error. Block for 100 ms
    LED::led_on(DEV_BOARD_LED_CAN);  // LED 2 on now

    /// Complete Period 1
    LED::green_on();  // LED Green on now

    /// Setup On-Board AHRS
    Vector3D ahrs_bias;

    ahrs.load_calibration_data(MPU6500_STORED_GYRO_BIAS);
    LOG_WARN("Use default AHRS bias");

    ahrs.start(ON_BOARD_AHRS_MATRIX_, THREAD_AHRS_PRIO);
    while(!ahrs.ready()) {
        chThdSleepMilliseconds(5);
    }
    //Inspector::startup_check_mpu();  // check MPU6500 has signal. Block for 20 ms
    //Inspector::startup_check_ist();  // check IST8310 has signal. Block for 20 ms
    Shell::addCommands(ahrs.shellCommands);
    Shell::addFeedbackCallback(AHRSOnBoard::cmdFeedback, &ahrs);
    LED::led_on(DEV_BOARD_LED_AHRS);  // LED 3 on now


    /// Setup MOTOR
    can_motor_interface::init(&can1, &can2);
    chThdSleepMilliseconds(2000);
    // FIXME: revert for development

    // Inspector::startup_check_gimbal_feedback(); // check gimbal motors has continuous feedback. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_GIMBAL);  // LED 5 on now


    /// Setup Red Spot Laser
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser

    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Echo Gimbal Raws and Converted Angles
//    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
//        GimbalIF::feedback[GimbalIF::YAW]->last_angle_raw, GimbalIF::feedback[GimbalIF::YAW]->actual_angle,
//        GimbalIF::feedback[GimbalIF::PITCH]->last_angle_raw, GimbalIF::feedback[GimbalIF::PITCH]->actual_angle);

    /// Start SKDs


    /// Start Inspector and User Threads
//    InspectorI::start_inspection(THREAD_INSPECTOR_PRIO);


    /// Complete Period 2
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_intel);  // Now play the startup sound


    /*** ------------------------ Period 3. End of main thread ----------------------- ***/

    // Entering empty loop with low priority
#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}