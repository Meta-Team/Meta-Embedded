//
// Created by liuzikai on 2019-01-27.
// Edited by Qian Chen & Mo Kanya on 2019-07-05
//

/// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer.h"
#include "common_macro.h"

#include "shell.h"
#include "can_interface.h"
#include "ahrs.h"
#include "remote_interpreter.h"

#include "gimbal_interface.h"
#include "gimbal_scheduler.h"
#include "shoot_scheduler.h"
#include "gimbal_logic.h"
#include "hero_shoot_logic.h"

#include "chassis_interface.h"
#include "chassis_scheduler.h"
#include "chassis_logic.h"

#include "inspector.h"
#include "user.h"

/// Vehicle Specific Configurations
#include "vehicle_hero.h"

/// Board Guard
#if defined(BOARD_RM_2018_A)
#else
#error "Hero supports only RM Board 2018 A currently"
#endif

/// Instances
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard ahrs;


int main() {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    // Enable power of bullet loader motor
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    /// Preparation of Period 1
    Inspector::init(&can1, &can2, &ahrs);
    LED::all_off();

    /// Setup Shell
    Shell::start(THREAD_SHELL_PRIO);
    LED::led_on(1);  // LED 1 on now

    /// Setup CAN1 & CAN2
    can1.start(THREAD_CAN1_PRIO);
    can2.start(THREAD_CAN2_PRIO);
    chThdSleepMilliseconds(5);
    Inspector::startup_check_can();  // check no persistent CAN Error. Block for 100 ms
    LED::led_on(2);  // LED 2 on now

    /// Setup On-Board AHRS
    ahrs.load_calibration_data(MPU6500_STORED_GYRO_BIAS);
    ahrs.start(ON_BOARD_AHRS_MATRIX, THREAD_MPU_PRIO, THREAD_IST_PRIO, THREAD_AHRS_PRIO);
    chThdSleepMilliseconds(5);
    Inspector::startup_check_mpu();  // check MPU6500 has signal. Block for 20 ms
    Inspector::startup_check_ist();  // check IST8310 has signal. Block for 20 ms
    LED::led_on(3);  // LED 3 on now

    /// Setup GimbalIF (for Gimbal and Shoot)
    GimbalIF::init(&can1, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);
    chThdSleepMilliseconds(10);
    Inspector::startup_check_gimbal_feedback(); // check gimbal motors has continuous feedback. Block for 20 ms
    LED::led_on(4);  // LED 5 on now


    /// Setup ChassisIF
    ChassisIF::init(&can2);
    chThdSleepMilliseconds(10);
    Inspector::startup_check_chassis_feedback();  // check chassis motors has continuous feedback. Block for 20 ms
    LED::led_on(5);  // LED 6 on now


    /// Setup Red Spot Laser
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser


    /// Complete Period 1
    LED::green_on(); // LED Green on now



    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Echo Gimbal Raws and Converted Angles
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        GimbalIF::feedback[GimbalIF::YAW].last_angle_raw, GimbalIF::feedback[GimbalIF::YAW].actual_angle,
        GimbalIF::feedback[GimbalIF::PITCH].last_angle_raw, GimbalIF::feedback[GimbalIF::PITCH].actual_angle);

    /// Start SKDs
    GimbalSKD::start(&ahrs, GIMBAL_ANGLE_INSTALLATION_MATRIX, GIMBAL_GYRO_INSTALLATION_MATRIX,
                     GIMBAL_YAW_INSTALL_DIRECTION, GIMBAL_PITCH_INSTALL_DIRECTION, THREAD_GIMBAL_SKD_PRIO);
    GimbalSKD::load_pid_params(GIMBAL_PID_YAW_A2V_PARAMS, GIMBAL_PID_YAW_V2I_PARAMS,
                               GIMBAL_PID_PITCH_A2V_PARAMS, GIMBAL_PID_PITCH_V2I_PARAMS);

    ShootSKD::start(SHOOT_BULLET_INSTALL_DIRECTION, ShootSKD::POSITIVE /* of no use */, THREAD_SHOOT_SKD_PRIO);
    ShootSKD::load_pid_params(SHOOT_PID_BULLET_LOADER_A2V_PARAMS, SHOOT_PID_BULLET_LOADER_V2I_PARAMS,
                              SHOOT_PID_BULLET_PLATE_A2V_PARAMS, SHOOT_PID_BULLET_PLATE_V2I_PARAMS);

    ChassisSKD::start(CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE, THREAD_CHASSIS_SKD_PRIO);
    ChassisSKD::load_pid_params(CHASSIS_PID_THETA2V_PARAMS, CHASSIS_PID_V2I_PARAMS);

    /// Start LGs
    // GimbalLG does not need initialization
    HeroShootLG::init(LOADER_SHOOT_DEGREE_PER_BULLET, PLATE_SHOOT_DEGREE_PER_BULLET, THREAD_SHOOT_LG_STUCK_DETECT_PRIO, THREAD_SHOOT_LG_AUTOMATION_PRIO);
    ChassisLG::init(THREAD_CHASSIS_LG_DODGE_PRIO);


    /// Start Inspector and User Threads
    Inspector::start_inspection(THREAD_INSPECTOR_PRIO);
    User::start(THREAD_USER_PRIO);

    /// Complete Period 2
    Buzzer::play_sound(Buzzer::sound_startup_intel, THREAD_BUZZER_PRIO);  // Now play the startup sound


    /*** ------------------------ Period 3. End of main thread ----------------------- ***/

    // Entering empty loop with low priority
#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(THREAD_IDEAL_PRIO);
#endif
    return 0;
}