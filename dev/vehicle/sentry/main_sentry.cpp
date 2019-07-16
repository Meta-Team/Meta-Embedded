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
 *  MID   DOWN  Remote - Various speed mode, used to test the final auto mode
 *  DOWN  UP/MID Auto (temporarily this can't be achieved, so just left it be the Safe mode)
 *  DOWN  DOWN  Final Auto Mode, Random walk.
 *  -Others-    Safe
 * ------------------------------------------------------------
 */

CANInterface can1(&CAND1);
//AHRSExt ahrsExt;

/** Threads **/

/**
 * @name GimbalThread
 * @brief Thread to control gimbal, including shooting mechanism
 * @pre Remote interpreter starts receive
 * @pre Initialize GimbalInterface with CAN driver and set the front angles of yaw and pitch properly
 * @pre Start the thread of updating MPU6500
 */
class SentryThread : public chibios_rt::BaseStaticThread<512> {

    static constexpr unsigned int GIMBAL_THREAD_INTERVAL = 10; // [ms]

    void main() final {
        setName("sentry");
        // bool escaping = false;
        bool under_attack;
        Remote::rc_status_t s1_present_state = Remote::S_UP, s2_present_state = Remote::S_UP;
        while (!shouldTerminate()) {

            /** Setting State **/

            if (s1_present_state != Remote::rc.s1 || s2_present_state != Remote::rc.s2) {
                // If the state of remote controller is changed, then we change the state/mode of the SKDs
                s1_present_state = Remote::rc.s1;
                s2_present_state = Remote::rc.s2;

                switch (s1_present_state) {

                    case Remote::S_UP :

                        SChassisSKD::turn_off();

                        switch (s2_present_state) {

                            case Remote::S_UP :
                                SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::YAW_ID, false);
                                SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::PIT_ID, false);
                                SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::BULLET_LOADER_ID, false);
                                SuspensionGimbalSKD::set_shoot_mode(OFF);
                                break;
                            case Remote::S_MIDDLE :
                                LED::led_toggle(7);
                                SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::YAW_ID, true);
                                SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::PIT_ID, true);
                                SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::BULLET_LOADER_ID, true);
                                SuspensionGimbalSKD::set_shoot_mode(AWAIT);
                                SuspensionGimbalSKD::set_front(SuspensionGimbalIF::YAW_ID);
                                SuspensionGimbalSKD::set_front(SuspensionGimbalIF::PIT_ID);
                                SuspensionGimbalSKD::set_front(SuspensionGimbalIF::BULLET_LOADER_ID);
                                break;
                            case Remote::S_DOWN :
                                SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::YAW_ID, true);
                                SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::PIT_ID, true);
                                SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::BULLET_LOADER_ID, true);
                                SuspensionGimbalSKD::set_shoot_mode(AWAIT);
                                SuspensionGimbalSKD::set_front(SuspensionGimbalIF::YAW_ID);
                                SuspensionGimbalSKD::set_front(SuspensionGimbalIF::PIT_ID);
                                SuspensionGimbalSKD::set_front(SuspensionGimbalIF::BULLET_LOADER_ID);
                                break;
                        }

                        break;

                    case Remote::S_MIDDLE :

                        SChassisSKD::turn_on();
                        SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::YAW_ID, false);
                        SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::PIT_ID, false);
                        SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::BULLET_LOADER_ID, false);
                        SuspensionGimbalSKD::set_shoot_mode(OFF);

                        switch (s2_present_state) {
                            case Remote::S_UP :
                                SChassisSKD::set_mode(SChassisSKD::ONE_STEP_MODE);
                                break;
                            case Remote::S_MIDDLE :
                                SChassisSKD::set_mode(SChassisSKD::SHUTTLED_MODE);
                                break;
                            case Remote::S_DOWN :
                                SChassisSKD::set_mode(SChassisSKD::FINAL_AUTO_MODE);
                                break;
                        }
                        break;
                    case Remote::S_DOWN :
                        SChassisSKD::turn_off();
                        SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::YAW_ID, false);
                        SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::PIT_ID, false);
                        SuspensionGimbalSKD::set_motor_enable(SuspensionGimbalIF::BULLET_LOADER_ID, false);
                        SuspensionGimbalSKD::set_shoot_mode(OFF);
                        switch (s2_present_state) {
                            case Remote::S_UP :
                                break;
                            case Remote::S_MIDDLE :
                                break;
                            case Remote::S_DOWN :
                                break;
                        }
                        break;
                }

            }


            /** Update Movement Request **/
            if (s1_present_state == Remote::S_UP && s2_present_state == Remote::S_UP) {
                //     LOG("%.2f, %.2f", SuspensionGimbalIF::yaw.angular_position, SuspensionGimbalIF::pitch.angular_position);
            } else if (s1_present_state == Remote::S_UP && s2_present_state == Remote::S_MIDDLE) {
                SuspensionGimbalSKD::set_motor_angle(SuspensionGimbalIF::YAW_ID, Remote::rc.ch2 * 100.0f);
                SuspensionGimbalSKD::set_motor_angle(SuspensionGimbalIF::PIT_ID, Remote::rc.ch3 * 40);
                if (Remote::rc.ch0 > 0.5f) {
                    SuspensionGimbalSKD::start_continuous_shooting();
                } else {
                    SuspensionGimbalSKD::stop_continuous_shooting();
                }
                //   LOG("%.2f, %.2f", Remote::rc.ch2 * 170.0f, Remote::rc.ch3 * 40.0f);
            } else if (s1_present_state == Remote::S_MIDDLE && s2_present_state == Remote::S_UP) {
                SChassisSKD::set_destination(SChassisIF::  + Remote::rc.ch0);
                //  LOG("%.2f", SChassisIF::present_position);
            } else if (s1_present_state == Remote::S_MIDDLE && s2_present_state == Remote::S_DOWN) {
                /// FINAL_AUTO_MODE, random escape
                // if not escaping but under attack, go into escape mode, use to gimbal data when gimbal is connected
                // under_attack = Remote::rc.ch2>=0.5 || Remote::rc.ch2<=-0.5;
                under_attack = true;
                if (under_attack) SChassisSKD::start_escaping();
                LOG("chassis power: %.2f", Referee::power_heat_data.chassis_power);
            }
            //LOG("%d",Referee::count_);
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
    Referee::init();
    LOG("3");
    Remote::start();
    LOG("4");


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /*** Parameters Set up***/
    // ahrsExt.start(&can1);
    // SuspensionGimbalIF::init(&can1, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);
    SChassisIF::init(&can1);
    LOG("5");
    // SuspensionGimbalSKD::init(&ahrsExt);
    // SuspensionGimbalSKD::suspensionGimbalThread.start(HIGHPRIO - 2);
    SChassisSKD::sentryChassisThread.start(HIGHPRIO - 3);

    LED::green_on();
    /** Start Logic Control Thread **/
    chThdSleepMilliseconds(500);
    sentryThread.start(NORMALPRIO);
    /** Echo Gimbal Raws and Converted Angles **/
    chThdSleepMilliseconds(500);
    /*LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        SuspensionGimbalIF::yaw.last_angle, SuspensionGimbalIF::yaw.angular_position,
        SuspensionGimbalIF::pitch.last_angle, SuspensionGimbalIF::pitch.angular_position);
    */
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