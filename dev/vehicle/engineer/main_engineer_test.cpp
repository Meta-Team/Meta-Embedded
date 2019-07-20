//
// Created by Kerui Zhu on 7/10/2019.
// Modified by LaiXinyi on 7/19/2019.
//


#include "ch.hpp"
#include "hal.h"

#include "vehicle_engineer.h"

#include "can_interface.h"
#include "common_macro.h"

#include "buzzer.h"
#include "remote_interpreter.h"

#include "engineer_chassis_interface.h"
#include "engineer_elevator_interface.h"
#include "robotic_arm_interface.h"

#include "engineer_chassis_skd.h"
#include "engineer_elevator_skd.h"
#include "robotic_arm_skd.h"


#include "engineer_elevator_logic.h"

/**
 * Mode Table:
 * ------------------------------------------------------------
 * Left  Right  Mode
 * ------------------------------------------------------------
 *  UP    UP    Safe
 *  UP    MID   Remote - Chassis remote controlling
 *  UP    DOWN  Remote - Elevator remote controlling
 *  MID   UP    Remote - Auto elevating
 *  MID   MID   Remote - Auto Fetching Bullet /// not done yet
 *  MID   DOWN  Remote - Robotic arm test
 *  DOWN  UP    ***
 *  DOWN  MID   ***
 *  DOWN  DOWN  Final PC MODE
 *  -Others-    Safe
 * ------------------------------------------------------------
 */

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

class EngineerThread: public chibios_rt::BaseStaticThread<1024>{

    static constexpr unsigned int ENGINEER_THREAD_INTERVAL = 10; // [ms]

    void main() final{
        setName("engineer");

        while (!shouldTerminate()){

            /// safe mode
            if ( Remote::rc.s1 == Remote::S_UP && Remote::rc.s2 == Remote::S_UP ) {
                EngineerChassisSKD::lock();
                EngineerElevatorLG::set_action_lock();
                // and disable Robotic arm
            }

            /// remote chassis, left FBLR, right turn
            else if ( Remote::rc.s1 == Remote::S_UP && Remote::rc.s2 == Remote::S_MIDDLE ) {
                EngineerChassisSKD::unlock();
                EngineerElevatorLG::set_action_lock();
                // and disable Robotic arm
                EngineerChassisSKD::set_velocity(
                        Remote::rc.ch2 * ENGINEER_CHASSIS_VELOCITY_MAX,
                        Remote::rc.ch3 * ENGINEER_CHASSIS_VELOCITY_MAX,
                        -Remote::rc.ch0 * ENGINEER_CHASSIS_W_MAX);
            }

            /// remote elevator, left aided motor, right elevator
            else if ( Remote::rc.s1 == Remote::S_UP && Remote::rc.s2 == Remote::S_DOWN ) {
                EngineerChassisSKD::lock();
                EngineerElevatorLG::set_action_free();
                // and disable Robotic arm
                if ( Remote::rc.ch1 > 0.5 || Remote::rc.ch1 < -0.5 ) {
                    EngineerElevatorSKD::elevator_enable(true);
                    EngineerElevatorSKD::aided_motor_enable(false);
                    EngineerElevatorSKD::set_target_height( EngineerElevatorIF::get_current_height() + Remote::rc.ch1 * 2 );
                }
                else if ( Remote::rc.ch3 > 0.2 || Remote::rc.ch3 < -0.2 ) {
                    EngineerElevatorSKD::elevator_enable(false);
                    EngineerElevatorSKD::aided_motor_enable(true);
                    EngineerElevatorSKD::set_aided_motor_velocity( Remote::rc.ch3 * ENGINEER_AIDED_MOTOR_VELOCITY, Remote::rc.ch3 * ENGINEER_AIDED_MOTOR_VELOCITY );
                }
                else if ( Remote::rc.ch2 > 0.5 ) {
                    EngineerElevatorSKD::elevator_enable(false);
                    EngineerElevatorSKD::aided_motor_enable(true);
                    EngineerElevatorSKD::set_aided_motor_velocity( Remote::rc.ch3 * ENGINEER_AIDED_MOTOR_VELOCITY, 0);
                }
                else if ( Remote::rc.ch2 < -0.5 ) {
                    EngineerElevatorSKD::elevator_enable(false);
                    EngineerElevatorSKD::aided_motor_enable(true);
                    EngineerElevatorSKD::set_aided_motor_velocity( 0, Remote::rc.ch3 * ENGINEER_AIDED_MOTOR_VELOCITY);
                }
            }


            /// remote auto elevating. RIGHT.
            ///                               |-UP -> nothing happens
            ///                               |
            ///       |-UP -> start going-up -|                |-UP -> con't
            ///       |                       |-DOWN -> pause -|
            ///       |                                        |-DOWN -> quit
            /// stop -|
            ///       |                                          |-UP -> quit
            ///       |                           |-UP -> pause -|
            ///       |-DOWN -> start going-down -|              |-DOWN -> con't
            ///                                   |
            ///                                   |-DOWN -> nothing happens
            ///
            else if ( Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP ) {

                if ( EngineerElevatorLG::get_action() == EngineerElevatorLG::LOCK ) {
                    if (Remote::rc.ch1 > 0.5)
                        EngineerElevatorLG::start_going_up();
                    else if (Remote::rc.ch2 < -0.5)
                        EngineerElevatorLG::start_going_down();
                }

                else if ( EngineerElevatorLG::get_action() == EngineerElevatorLG::UPWARD ) {
                    if (Remote::rc.ch2 < -0.5)
                        EngineerElevatorLG::pause_action();
                }
                else if ( EngineerElevatorLG::get_action() == EngineerElevatorLG::DOWNWARD ) {
                    if (Remote::rc.ch2 < 0.5)
                        EngineerElevatorLG::pause_action();
                }

                else if ( EngineerElevatorLG::get_action() == EngineerElevatorLG::PAUSE ) {
                    if ( ( EngineerElevatorLG::get_prev_action() == EngineerElevatorLG::UPWARD && Remote::rc.ch1 > 0.5 ) ||
                         ( EngineerElevatorLG::get_prev_action() == EngineerElevatorLG::DOWNWARD && Remote::rc.ch1 < -0.5 ) )
                        EngineerElevatorLG::continue_action();
                    else if ( ( EngineerElevatorLG::get_prev_action() == EngineerElevatorLG::UPWARD && Remote::rc.ch1 < -0.5 ) ||
                              ( EngineerElevatorLG::get_prev_action() == EngineerElevatorLG::DOWNWARD && Remote::rc.ch1 > 0.5 ) )
                        EngineerElevatorLG::quit_action();
                }

            }


            /// remote auto fetching bullet - NOT DONE YET
            else if ( Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE ) {
                // set as safe currently
                EngineerChassisSKD::lock();
                EngineerElevatorLG::set_action_lock();
            }

            /// remote Robotic arm test
            else if ( Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN ) {

                if (Remote::rc.ch1 > 0.5) RoboticArmSKD::set_status(RoboticArmSKD::lift_state, LIFT_PAD, RoboticArmSKD::HIGH_STATUS);
                else if (Remote::rc.ch1 < -0.5) RoboticArmSKD::set_status(RoboticArmSKD::lift_state, LIFT_PAD, RoboticArmSKD::LOW_STATUS);
                else if (Remote::rc.ch0 > 0.5) RoboticArmSKD::set_status(RoboticArmSKD::extend_state, EXTEND_PAD, RoboticArmSKD::HIGH_STATUS);
                else if (Remote::rc.ch0 < -0.5) RoboticArmSKD::set_status(RoboticArmSKD::extend_state, EXTEND_PAD, RoboticArmSKD::LOW_STATUS);
                else if (Remote::rc.ch2 > 0.5) RoboticArmSKD::change_status(RoboticArmSKD::door_state, DOOR_PAD);
                else if (Remote::rc.ch3 > 0.5) RoboticArmSKD::set_clamp_action(RoboticArmSKD::CLAMP_CLAMPED);
                else if (Remote::rc.ch3 < -0.5) RoboticArmSKD::set_clamp_action(RoboticArmSKD::CLAMP_RELAX);
                else if (Remote::rc.ch2 < -0.5) RoboticArmSKD::pull_back();

            }


            /// safe
            else if ( Remote::rc.s1 == Remote::S_DOWN && Remote::rc.s2 != Remote::S_DOWN ) {
                // the same as UP-UP
                EngineerChassisSKD::lock();
                EngineerElevatorLG::set_action_lock();
                // and disable Robotic arm
            }


            /// PC control
            else if ( Remote::rc.s1 == Remote::S_DOWN && Remote::rc.s2 == Remote::S_DOWN ) {

                /// Chassis

                /// Chassis Config
                float chassis_v_left_right = 1000.0f;  // [mm/s]
                float chassis_v_forward = 3000.0f;     // [mm/s]
                float chassis_v_backward = 3000.0f;    // [mm/s]

                float chassis_pc_shift_ratio = 1.5f;  // 150% when Shift is pressed
                float chassis_pc_ctrl_ratio = 0.5;    // 50% when Ctrl is pressed

                float target_vx, target_vy;

                if (Remote::key.w)      target_vy = chassis_v_forward;
                else if (Remote::key.s) target_vy = -chassis_v_backward;
                else                    target_vy = 0;

                if (Remote::key.d)      target_vx = chassis_v_left_right;
                else if (Remote::key.a) target_vx = -chassis_v_left_right;
                else                    target_vx = 0;

                if (Remote::key.ctrl) {
                    target_vx *= chassis_pc_ctrl_ratio;
                    target_vy *= chassis_pc_ctrl_ratio;
                } else if (Remote::key.shift) {
                    target_vx *= chassis_pc_shift_ratio;
                    target_vy *= chassis_pc_shift_ratio;
                }
                // No need to echo to user since it has been done above

                ChassisLG::set_target(target_vx, target_vy);


                /// Elevator

                Remote::key_t UserI::chassis_dodge_switch = Remote::KEY_X;


                /// Robotic Arm


            }



            sleep(TIME_MS2I(ENGINEER_THREAD_INTERVAL));
        }
    }
}engineerThread;

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
    can2.start(HIGHPRIO - 2);
    Remote::start();


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /*** Parameters Set up***/
    EngineerChassisIF::init(&can1);
    EngineerElevatorIF::init(&can2);
    RoboticArmIF::init(&can2);

    //TODO ???
    EngineerChassisSKD::engineerChassisThread.start(HIGHPRIO - 2);
    EngineerElevatorSKD::engineerElevatorThread.start(HIGHPRIO - 3);
    RoboticArmSKD::roboticArmThread.start(HIGHPRIO - 4);
    EngineerElevatorLG::engineerLogicThread.start(HIGHPRIO - 5);

    LED::green_on();
    /** Start Logic Control Thread **/
    chThdSleepMilliseconds(500);
    engineerThread.start(NORMALPRIO);
    /** Echo Gimbal Raws and Converted Angles **/
    chThdSleepMilliseconds(500);
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