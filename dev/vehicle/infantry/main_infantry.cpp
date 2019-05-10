//
// Created by liuzikai on 2019-01-27.
//

// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"

#include "can_interface.h"
#include "common_macro.h"

#include "buzzer.h"
#include "mpu6500.h"
#include "remote_interpreter.h"

#include "gimbal.h"
#include "shoot.h"
#include "chassis.h"

#include "state_handler.h"

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
#error "RoboticArm interface is only developed for RM board 2018 A."
#endif

// Threads code
#include "thread_gimbal.hpp"
#include "thread_shoot.hpp"
#include "thread_chassis.hpp"

// Interfaces
CANInterface can1(&CAND1);

// Threads
GimbalThread gimbalThread;
ShootThread ShootThread;
ChassisThread chassisThread;


int main(void) {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    /*** --------------------------- Period 1. Modules Setup --------------------------- ***/

    LED::all_off();

    time_msecs_t t;  // variable for startup check

    Shell::start(HIGHPRIO);
    // LED 1 on now


    can1.start(HIGHPRIO - 1);
    // Check no persistent CAN Error (100 ms)
    t = SYSTIME;
    while (SYSTIME - t < 100) {
        if (StateHandler::fetchCANErrorMark()) {  // can error occurs
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
    StateHandler::echoEvent(StateHandler::CAN_START_SUCCESSFULLY);
    // LED 2 on now


    MPU6500::start(HIGHPRIO - 2);
    // Check MPU6500 has signal. Block for 10 ms
    t = SYSTIME;
    while (SYSTIME - t < 20) {
        if (SYSTIME - MPU6500::last_update_time > 3) {
            // No signal in last 3 ms (normal interval 1 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(3);
    }
    StateHandler::echoEvent(StateHandler::MPU6500_START_SUCCESSFULLY);
    // LED 3 on now


    Remote::start_receive();
    // Check Remote has signal. Block for 50 ms
    t = SYSTIME;
    while (SYSTIME - t < 50) {
        if (SYSTIME - Remote::last_update_time > 15) {
            // No signal in last 15 ms (normal interval 7 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(15);
    }
    StateHandler::echoEvent(StateHandler::REMOTE_START_SUCCESSFULLY);
    // LED 4 on now


    Gimbal::init(&can1, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);
    chThdSleepMilliseconds(10);  // wait for 10 ms
    // Check gimbal motors has continuous feedback. Block for 50 ms
    // TODO: echo to user which motor lose connection
    t = SYSTIME;
    while (SYSTIME - t < 50) {
        if (SYSTIME - Gimbal::feedback[Gimbal::YAW].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - Gimbal::feedback[Gimbal::PITCH].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - Gimbal::feedback[Gimbal::BULLET].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(3);
    }
    StateHandler::echoEvent(StateHandler::GIMBAL_CONNECTED);
    // LED 5 on now


    Chassis::init(&can1, CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE);
    chThdSleepMilliseconds(10);  // wait for 10 ms
    // Check chassis motors has continuous feedback. Block for 50 ms
    // TODO: echo to user which motor lose connection
    t = SYSTIME;
    while (SYSTIME - t < 50) {
        if (SYSTIME - Chassis::feedback[Chassis::CHASSIS_FR].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - Chassis::feedback[Chassis::CHASSIS_FL].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - Chassis::feedback[Chassis::CHASSIS_BL].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - Chassis::feedback[Chassis::CHASSIS_BR].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(3);
    }
    StateHandler::echoEvent(StateHandler::CHASSIS_CONNECTED);
    // LED 6 on now

    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // Enable the red spot laser

    StateHandler::echoEvent(StateHandler::MAIN_MODULES_SETUP_COMMPLETE);
    // LED Green on now

    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /** Echo Gimbal Raws and Converted Angles **/
    chThdSleepMilliseconds(500);
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        Gimbal::feedback[Gimbal::YAW].last_angle_raw, Gimbal::feedback[Gimbal::YAW].actual_angle,
        Gimbal::feedback[Gimbal::PITCH].last_angle_raw, Gimbal::feedback[Gimbal::PITCH].actual_angle);

    /** Start Logic Control Thread **/
    gimbalThread.start(NORMALPRIO);
    chassisThread.start(NORMALPRIO - 1);

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