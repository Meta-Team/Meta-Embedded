//
// Created by zhukerui on 2019/5/18.
//

#include "ch.hpp"
#include "hal.h"

#include "vehicle_sentry.h"

// Modules and basic communication channels
#include "can_interface.h"
#include "common_macro.h"

// Interfaces
#include "buzzer.h"
#include "mpu6500.h"
#include "remote_interpreter.h"
#include "suspension_gimbal_interface.h"
#include "sentry_chassis_interface.h"

// Controllers
#include "suspension_gimbal_skd.h"
#include "sentry_chassis_skd.h"


/**
 * Mode Table:
 * ------------------------------------------------------------
 * Left  Right  Mode
 * ------------------------------------------------------------
 *  UP    UP    Safe
 *  UP    MID   Remote - Chassis safe; gimbal remote controlling
 *  UP    DOWN  Auto - Chassis safe; gimbal auto controlling
 *  MID   UP    Remote - Chassis remote controlling, constant speed mode; gimbal fix
 *  MID   MID   Remote - Constant speed mode
 *  MID   DOWN  Remote - Various speed mode
 *  DOWN  *     Auto (temporarily this can't be achieved, so just left it be the Safe mode)
 *  -Others-    Safe
 * ------------------------------------------------------------
 */

CANInterface can1(&CAND1);

/** Threads **/

/**
 * @name GimbalThread
 * @brief Thread to control gimbal, including shooting mechanism
 * @pre Remote interpreter starts receive
 * @pre Initialize GimbalInterface with CAN driver and set the front angles of yaw and pitch properly
 * @pre Start the thread of updating MPU6500
 */
class SentryThread : public chibios_rt::BaseStaticThread<1024> {

    static constexpr unsigned int GIMBAL_THREAD_INTERVAL = 10; // [ms]

    static constexpr float PC_YAW_SPEED_RATIO = 54000; // rotation speed when mouse moves at the fastest limit [degree/s]
    static constexpr float PC_YAW_ANGLE_LIMITATION = 60; // maximum range (both CW and CCW) [degree]

    static constexpr float PC_PITCH_SPEED_RATIO = 12000; // rotation speed when mouse moves at the fastest limit [degree/s]
    static constexpr float PC_PITCH_ANGLE_LIMITATION = 25; // maximum range (both up and down) [degree]

    void main() final {
        setName("sentry");

        /*** Parameters Set up***/

        /* Suspension Gimbal */
        SuspensionGimbalSKD::yaw_v_to_i.change_parameters(GIMBAL_PID_YAW_V2I_PARAMS);
        SuspensionGimbalSKD::yaw_angle_to_v.change_parameters(GIMBAL_PID_YAW_A2V_PARAMS);
        SuspensionGimbalSKD::pitch_v_to_i.change_parameters(GIMBAL_PID_PITCH_V2I_PARAMS);
        SuspensionGimbalSKD::pitch_angle_to_v.change_parameters(GIMBAL_PID_PITCH_A2V_PARAMS);
        SuspensionGimbalSKD::BL_v_to_i.change_parameters(GIMBAL_PID_BULLET_LOADER_V2I_PARAMS);
        SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::YAW_ID, true);
        SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::PIT_ID, true);
        SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::BULLET_LOADER_ID, true);
        SuspensionGimbalSKD::set_shoot_mode(SuspensionGimbalIF::OFF);

        /* Sentry Chassis */
        SentryChassisSKD::sentry_a2v_pid.change_parameters(SENTRY_CHASSIS_PID_A2V_PARAMS);
        SentryChassisSKD::left_v2i_pid.change_parameters(SENTRY_CHASSIS_PID_V2I_PARAMS);
        SentryChassisSKD::right_v2i_pid.change_parameters(SENTRY_CHASSIS_PID_V2I_PARAMS);
        SentryChassisSKD::set_mode(SentryChassisSKD::STOP_MODE);

        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::S_UP) {

            }

            // Bullet loader part


            SuspensionGimbalIF::send_gimbal_currents();

            sleep(TIME_MS2I(GIMBAL_THREAD_INTERVAL));
        }
    }
} sentryThread;


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
    //MPU6500Controller::start(HIGHPRIO - 2);
    Remote::start_receive();

    SuspensionGimbalIF::init(&can1, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);
    SentryChassisSKD::init(&can1);


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/


    LED::green_on();

    // Start the red spot
    /*
    palSetPadMode(GPIOG, GPIOG_PIN13, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOG, GPIOG_PIN13);
     */

    /** Echo Gimbal Raws and Converted Angles **/
    chThdSleepMilliseconds(500);
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        SuspensionGimbalIF::yaw.last_angle_raw, SuspensionGimbalIF::yaw.actual_angle,
        SuspensionGimbalIF::pitch.last_angle_raw, SuspensionGimbalIF::pitch.actual_angle);

    /** Start Logic Control Thread **/
    sentryThread.start(NORMALPRIO);

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