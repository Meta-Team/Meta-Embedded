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
    MPU6500::start(HIGHPRIO - 2);
    Remote::start_receive();

    GimbalInterface::init(&can1, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);
    Chassis::init(&can1, CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE);


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

    // Start the red spot
    palSetPadMode(GPIOG, GPIOG_PIN13, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOG, GPIOG_PIN13);

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