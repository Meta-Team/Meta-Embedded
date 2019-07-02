//
// Created by 404 on 2019-05-18.
//

// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#include "can_interface.h"
#include "common_macro.h"

#include "buzzer.h"
#include "interface/ahrs/mpu6500.h"
#include "remote_interpreter.h"

#include "gimbal.h"
#include "scheduler/shoot_scheduler.h"
#include "chassis.h"

#include "state_handler.h"

//Vehicle specific config
#if defined(HERO)

#include "vehicle_hero.h"

#else

#error "main_hero.cpp should only be used for Hero."

#endif

// Board guard
#if defined(BOARD_RM_2018_A)
#else
#error "HERO is only developed for RM board 2018 A."
#endif

#include "thread_gimbal.hpp"
#include "thread_shoot.hpp"
#include "thread_chassis.hpp"
#include "thread_error_detect.hpp"

// Interfaces
CANInterface can1(&CAND1);

//Threads
GimbalThread gimbalThread;
ShootThread shootThread;
ChassisThread chassisThread;
ErrorDetectThread errorDetectThread;

int main(void){

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();
    palSetPad(GPIOH, GPIOH_POWER4_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    LED::all_off();

    /** Setup Shell */
    Shell::start(HIGHPRIO);
    StateHandler::echoEvent(StateHandler::SHELL_START);
    // LED 1 on now

    /** Setup CAN1 */
    can1.start(HIGHPRIO - 1);
    chThdSleepMilliseconds(50);
    startupCheckCAN();  // check no persistent CAN Error. Block for 100 ms
    StateHandler::echoEvent(StateHandler::CAN_START_SUCCESSFULLY);
    // LED 2 on now


    /** Setup MPU6500 */
    MPU6500::start(HIGHPRIO - 2);
    startupCheckMPU6500();  // check MPU6500 has signal. Block for 10 ms
    StateHandler::echoEvent(StateHandler::MPU6500_START_SUCCESSFULLY);
    // LED 3 on now


    /** Setup Remote */
    Remote::start();
    startupCheckRemote();  // check Remote has signal. Block for 50 ms
    StateHandler::echoEvent(StateHandler::REMOTE_START_SUCCESSFULLY);
    // LED 4 on now


    /** Setup Gimbal and Shoot */
    Gimbal::init(&can1, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);
    Shoot::init(SHOOT_DEGREE_PER_BULLET, SHOOT_DEGREE_PER_BULLER_PLATE);
    chThdSleepMilliseconds(10);
    startupCheckGimbalFeedback(); // check gimbal motors has continuous feedback. Block for 50 ms
    StateHandler::echoEvent(StateHandler::GIMBAL_CONNECTED);
    // LED 5 on now


    /** Setup Chassis */
    Chassis::init(&can1, CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE);
    chThdSleepMilliseconds(10);
    startupCheckChassisFeedback();  // check chassis motors has continuous feedback. Block for 50 ms
    StateHandler::echoEvent(StateHandler::CHASSIS_CONNECTED);
    // LED 6 on now

    /** Setup Red Spot Laser*/
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser


    StateHandler::echoEvent(StateHandler::MAIN_MODULES_SETUP_COMPLETE);
    // LED Green on now

    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /** Echo Gimbal Raws and Converted Angles **/
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        Gimbal::feedback[Gimbal::YAW].last_angle_raw, Gimbal::feedback[Gimbal::YAW].actual_angle,
        Gimbal::feedback[Gimbal::PITCH].last_angle_raw, Gimbal::feedback[Gimbal::PITCH].actual_angle);

    /** Start Threads **/
    gimbalThread.start(NORMALPRIO);
    chassisThread.start(NORMALPRIO - 1);
    shootThread.start(NORMALPRIO - 2);
    errorDetectThread.start(LOWPRIO + 1);

    StateHandler::echoEvent(StateHandler::MAIN_THREAD_SETUP_COMPLETE);
    // Now play the startup sound

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