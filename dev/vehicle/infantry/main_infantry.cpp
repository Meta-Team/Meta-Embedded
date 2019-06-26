//
// Created by liuzikai on 2019-01-27.
//

// Headers
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
#include "shoot_logic.h"

#include "chassis_interface.h"
#include "chassis_scheduler.h"
#include "chassis_logic.h"

#include "inspector.h"

// Vehicle specific config
#if defined(INFANTRY_THREE)                                                 /** Infantry #3 **/
#include "vehicle_infantry_three.h"
#elif defined(INFANTRY_FOUR)                                                /** Infantry #4 **/
#include "vehicle_infantry_four.h"
#elif defined(INFANTRY_FIVE)                                                /** Infantry #5 **/

#include "vehicle_infantry_five.h"

#else
#error "main_infantry.cpp should only be used for Infantry #3, #4, #5."
#endif

// Board guard
#if defined(BOARD_RM_2018_A)
#else
#error "Infantry supports only RM Board 2018 A currently"
#endif


CANInterface can1(&CAND1);
AHRSOnBoard ahrs;


int main(void) {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();



    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    /// Preparation of Period 1
    Inspector::init()
    LED::all_off();

    /// Setup Shell
    Shell::start(THREAD_SHELL_PRIO);
    LED::led_on(1);  // LED 1 on now

    /// Setup CAN1
    can1.start(THREAD_CAN1_PRIO);
    chThdSleepMilliseconds(5);
    Inspector::startup_check_can();  // check no persistent CAN Error. Block for 100 ms
    LED::led_on(2);  // LED 2 on now

    /// Setup On-Board AHRS
    ahrs.start(GIMBAL_AHRS_INSTALL_MATRIX, THREAD_MPU_PRIO, THREAD_IST_PRIO, THREAD_AHRS_PRIO);
    chThdSleepMilliseconds(5);
    Inspector::startup_check_mpu();  // check MPU6500 has signal. Block for 20 ms
    Inspector::startup_check_ist();  // check IST8310 has signal. Block for 20 ms
    LED::led_on(3);  // LED 3 on now

    /// Setup Remote
    Remote::start();
    Inspector::startup_check_remote();  // check Remote has signal. Block for 50 ms
    LED::led_on(4);  // LED 4 on now


    /// Setup GimbalIF (for Gimbal and Shoot)
    GimbalIF::init(&can1, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);
    chThdSleepMilliseconds(10);
    Inspector::startup_check_gimbal_feedback(); // check gimbal motors has continuous feedback. Block for 20 ms
    LED::led_on(5);  // LED 5 on now


    /// Setup ChassisIF
    ChassisIF::init(&can1);
    chThdSleepMilliseconds(10);
    Inspector::startup_check_chassis_feedback();  // check chassis motors has continuous feedback. Block for 20 ms
    LED::led_on(6);  // LED 6 on now


    /// Setup Red Spot Laser
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser


    LED::green_on(); // LED Green on now



    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Echo Gimbal Raws and Converted Angles
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        GimbalIF::feedback[GimbalIF::YAW].last_angle_raw, GimbalIF::feedback[GimbalIF::YAW].actual_angle,
        GimbalIF::feedback[GimbalIF::PITCH].last_angle_raw, GimbalIF::feedback[GimbalIF::PITCH].actual_angle);

    /// Start SKDs
    GimbalSKD::start(&ahrs, GIMBAL_AHRS_INSTALL_MATRIX,
                     GIMBAL_YAW_INSTALL_DIRECTION, GIMBAL_PITCH_INSTALL_DIRECTION, THREAD_GIMBAL_SKD_PRIO);
    GimbalSKD::load_pid_params(GIMBAL_PID_YAW_A2V_PARAMS, GIMBAL_PID_YAW_V2I_PARAMS,
                               GIMBAL_PID_PITCH_A2V_PARAMS, GIMBAL_PID_PITCH_V2I_PARAMS);

    ShootSKD::start(SHOOT_BULLET_INSTALL_DIRECTION, ShootSKD::POSITIVE /* of no use */, THREAD_SHOOT_SKD_PRIO);
    ShootSKD::load_pid_params(SHOOT_PID_BULLET_LOADER_A2V_PARAMS, SHOOT_PID_BULLET_LOADER_V2I_PARAMS,
                              {0, 0, 0, 0, 0} /* of no use */, {0, 0, 0, 0, 0} /* of no use */);

    ChassisSKD::start(CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE, THREAD_CHASSIS_SKD_PRIO);
    ChassisSKD::load_pid_params(CHASSIS_PID_THETA2V_PARAMS, CHASSIS_PID_V2I_PARAMS);

    /// Start LGs
    // GimbalLG does not need initialization
    ShootLG::init(SHOOT_DEGREE_PER_BULLER, THREAD_SHOOT_LG_STUCK_DETECT_PRIO);
    ChassisLG::init(THREAD_CHASSIS_LG_DODGE_PRIO);


    /// Start Inspector and User Threads


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
    chibios_rt::BaseThread::setPriority(1);
#endif
    return 0;
}