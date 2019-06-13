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
#include "suspension_gimbal_controller.h"
#include "sentry_chassis.h"


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
        SuspensionGimbalController::yaw_v_to_i.change_parameters(GIMBAL_PID_YAW_V2I_PARAMS);
        SuspensionGimbalController::yaw_angle_to_v.change_parameters(GIMBAL_PID_YAW_A2V_PARAMS);
        SuspensionGimbalController::pitch_v_to_i.change_parameters(GIMBAL_PID_PITCH_V2I_PARAMS);
        SuspensionGimbalController::pitch_angle_to_v.change_parameters(GIMBAL_PID_PITCH_A2V_PARAMS);
        SuspensionGimbalController::BL_v_to_i.change_parameters(GIMBAL_PID_BULLET_LOADER_V2I_PARAMS);
        SuspensionGimbalController::set_motor_enable(SuspensionGimbalIF::YAW_ID, true);
        SuspensionGimbalController::set_motor_enable(SuspensionGimbalIF::PIT_ID, true);
        SuspensionGimbalController::set_motor_enable(SuspensionGimbalIF::BULLET_LOADER_ID, true);
        SuspensionGimbalController::set_shoot_mode(SuspensionGimbalIF::OFF);

        /* Sentry Chassis */
        SentryChassisController::motor_left_pid.change_parameters(SENTRY_CHASSIS_PID_A2V_PARAMS);
        SentryChassisController::motor_right_pid.change_parameters(SENTRY_CHASSIS_PID_A2V_PARAMS);
        SentryChassisController::set_mode(SentryChassisController::STOP_MODE);

        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE){
                // Remote - Shooting (left) + Gimbal (right)

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

            } else{
                GimbalInterface::yaw.target_current = GimbalInterface::pitch.target_current = 0;
            }

            // Bullet loader part
            GimbalController::bullet_loader.update_accumulation_angle(
                    GimbalInterface::bullet_loader.actual_angle + GimbalInterface::bullet_loader.round_count * 360.0f);

            if (Remote::rc.s1 == Remote::RC_S_MIDDLE && Remote::rc.s2 == Remote::RC_S_UP){
                GimbalInterface::friction_wheels.duty_cycle = GIMBAL_REMOTE_FRICTION_WHEEL_DUTY_CYCLE;
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
            } else{
                GimbalInterface::friction_wheels.duty_cycle = 0;
                GimbalInterface::bullet_loader.target_current = 0;
            }

            GimbalInterface::send_gimbal_currents();

            sleep(TIME_MS2I(GIMBAL_THREAD_INTERVAL));
        }
    }
} sentryThread;

/**
 * @name ChassisThread
 * @brief thread to control chassis
 * @pre RemoteInterpreter start receive
 * @pre initialize ChassisInterface with CAN driver
 */
class ChassisThread : public chibios_rt::BaseStaticThread<1024> {


    void main() final {

        setName("sentry_chassis");

        SentryChassisController::change_v_to_i_pid(SENTRY_CHASSIS_PID_V2I_PARAMS);
        SentryChassisController::set_mode(SentryChassisController::STOP_MODE);
        Remote::rc_status_t previous_state_1 = Remote::rc.s1;
        Remote::rc_status_t previous_state_2 = Remote::rc.s2;
        SentryChassisController::enable = true;

        while (!shouldTerminate()) {

            if(previous_state_1 != Remote::rc.s1 || previous_state_2 != Remote::rc.s2){
                // If the remote state is changed, then we change the mode accordingly

                if(Remote::rc.s1 == Remote::RC_S_MIDDLE && Remote::rc.s2 == Remote::RC_S_MIDDLE){

                    // If it is changed to Remote - Constant speed mode

                    if(!(previous_state_1 == Remote::RC_S_MIDDLE && previous_state_2 == Remote::RC_S_DOWN))
                        // If it is not at the various speed mode previously, then we should set it to AUTO MODE and set the radius
                        SentryChassisController::set_mode(SentryChassisController::AUTO_MODE, 30);

                    SentryChassisController::change_speed_mode(false);

                } else if(Remote::rc.s1 == Remote::RC_S_MIDDLE && Remote::rc.s2 == Remote::RC_S_DOWN){

                    // If it is changed to Remote - Various speed mode

                    if(!(previous_state_1 == Remote::RC_S_MIDDLE && previous_state_2 == Remote::RC_S_MIDDLE))
                        // If it is not at the constant speed mode previously, then we should set it to AUTO MODE and set the radius
                        SentryChassisController::set_mode(SentryChassisController::AUTO_MODE, 30);

                    SentryChassisController::change_speed_mode(true);

                } else{
                    SentryChassisController::set_mode(SentryChassisController::STOP_MODE);
                }

                // Update the remote state
                previous_state_1 = Remote::rc.s1;
                previous_state_2 = Remote::rc.s2;
            }

            SentryChassisController::update_target_current();

            SentryChassisController::send_currents();

            sleep(TIME_MS2I(100));

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
    SentryChassisController::init(&can1);


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