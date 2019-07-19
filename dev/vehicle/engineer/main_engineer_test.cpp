//
// Created by Kerui Zhu on 7/10/2019.
// Modified by LaiXinyi on 7/10/2019.
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
 *  MID   UP    Remote - Auto Fetching Bullet /// not done yet
 *  MID   MID   Remote - Auto elevating
 *  MID   DOWN  Remote - Robotic arm
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
//        Remote::rc_status_t s1_present_state = Remote::S_UP, s2_present_state = Remote::S_UP;

        while (!shouldTerminate()){


            /// safe mode
            if ( Remote::rc.s1 == Remote::S_UP && Remote::rc.s2 == Remote::S_UP ) {
                EngineerChassisSKD::lock();
                EngineerElevatorLG::set_action_lock();
                //TODO disable Robotic arm
            }


            /// remote chassis, left FBLR, right turn
            else if ( Remote::rc.s1 == Remote::S_UP && Remote::rc.s2 == Remote::S_MIDDLE ) {
                EngineerChassisSKD::unlock();
                EngineerElevatorLG::set_action_lock();
                //TODO disable Robotic arm
                EngineerChassisSKD::set_velocity(
                        Remote::rc.ch2 * ENGINEER_CHASSIS_VELOCITY_MAX,
                        Remote::rc.ch3 * ENGINEER_CHASSIS_VELOCITY_MAX,
                        -Remote::rc.ch0 * ENGINEER_CHASSIS_W_MAX);
            }


            /// remote elevator, left aided motor, right elevator
            else if ( Remote::rc.s1 == Remote::S_UP && Remote::rc.s2 == Remote::S_DOWN ) {
                EngineerChassisSKD::lock();
                EngineerElevatorLG::set_action_free();
                //TODO disable Robotic arm
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




            /** Setting State **/

            if (s1_present_state != Remote::rc.s1 || s2_present_state != Remote::rc.s2){
                // If the state of remote controller is changed, then we change the state/mode of the SKDs
                s1_present_state = Remote::rc.s1;
                s2_present_state = Remote::rc.s2;

                switch (s1_present_state) {

                    case Remote::S_UP :
//                        switch (s2_present_state){
//
//                            case Remote::S_UP :
//                                EngineerChassisSKD::lock();
//                                EngineerElevatorSKD::elevator_enable(false);
//                                EngineerElevatorSKD::aided_motor_enable(false);
//                                break;
//                            case Remote::S_MIDDLE :
//                                EngineerChassisSKD::unlock();
//                                EngineerElevatorSKD::elevator_enable(false);
//                                EngineerElevatorSKD::aided_motor_enable(false);
//                                break;
//                            case Remote::S_DOWN :
//                                EngineerChassisSKD::lock();
//                                EngineerElevatorSKD::elevator_enable(true);
//                                EngineerElevatorSKD::aided_motor_enable(true);
//                                break;
//                        }
                        break;

                    case Remote::S_MIDDLE :
                        // The whole section is for a complete remote landing process
                        EngineerElevatorSKD::elevator_enable(true);
                        EngineerElevatorSKD::aided_motor_enable(true);
                        switch (s2_present_state){
                            case Remote::S_UP :
                                EngineerChassisSKD::unlock();
                                EngineerElevatorSKD::set_target_height(0);
                                break;
                            case Remote::S_MIDDLE :
                                EngineerChassisSKD::lock();
                                EngineerElevatorSKD::set_target_height(22);
                                elevating = true;
                                break;
                            case Remote::S_DOWN :
                                break;
                        }
                        break;
                    case Remote::S_DOWN :
                        switch (s2_present_state){
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

            switch (s1_present_state){
                case Remote::S_UP:
//                    switch (s2_present_state){
//                        case Remote::S_UP:
//                            break;
//                        case Remote::S_MIDDLE:
//                            EngineerChassisSKD::set_velocity(Remote::rc.ch2 * ENGINEER_CHASSIS_VELOCITY_MAX, Remote::rc.ch3 * ENGINEER_CHASSIS_VELOCITY_MAX, -Remote::rc.ch0 * ENGINEER_CHASSIS_W_MAX);
//                            break;
//                        case Remote::S_DOWN:
//                            EngineerElevatorSKD::set_target_height(EngineerElevatorSKD::target_height + Remote::rc.ch3 * 2);
//                            if (Remote::rc.ch0 > 0.5) EngineerElevatorSKD::set_aided_motor_velocity(Remote::rc.ch0 * ENGINEER_AIDED_MOTOR_VELOCITY, 0);
//                            else if (Remote::rc.ch0 < - 0.5) EngineerElevatorSKD::set_aided_motor_velocity(0, - Remote::rc.ch0 * ENGINEER_AIDED_MOTOR_VELOCITY);
//                            else EngineerElevatorSKD::set_aided_motor_velocity(Remote::rc.ch1 * ENGINEER_AIDED_MOTOR_VELOCITY, Remote::rc.ch1 * ENGINEER_AIDED_MOTOR_VELOCITY);
//                            break;
//                    }
                    break;
                case Remote::S_MIDDLE:
                    switch (s2_present_state){
                        case Remote::S_UP:
                            EngineerChassisSKD::set_velocity(Remote::rc.ch2 * ENGINEER_CHASSIS_VELOCITY_MAX, Remote::rc.ch3 * ENGINEER_CHASSIS_VELOCITY_MAX, Remote::rc.ch0 * ENGINEER_CHASSIS_W_MAX);
                            break;
                        case Remote::S_MIDDLE:
                            if (Remote::rc.ch0 > 0.5) EngineerElevatorSKD::set_aided_motor_velocity(Remote::rc.ch0 * ENGINEER_AIDED_MOTOR_VELOCITY, 0);
                            else if (Remote::rc.ch0 < - 0.5) EngineerElevatorSKD::set_aided_motor_velocity(0, - Remote::rc.ch0 * ENGINEER_AIDED_MOTOR_VELOCITY);
                            else EngineerElevatorSKD::set_aided_motor_velocity(Remote::rc.ch1 * ENGINEER_AIDED_MOTOR_VELOCITY, Remote::rc.ch1 * ENGINEER_AIDED_MOTOR_VELOCITY);

                            if (elevating && EngineerElevatorIF::elevatorMotor[0].present_angle > 20 * ANGLE_HEIGHT_RATIO){
                                elevating = false;
                            }
                            if (!elevating){
                                EngineerElevatorSKD::set_target_height(EngineerElevatorSKD::target_height + Remote::rc.ch3);
                            }
                        case Remote::S_DOWN:
                            if (Remote::rc.ch1 > 0.5) RoboticArmSKD::set_status(RoboticArmSKD::lift_state, LIFT_PAD, RoboticArmSKD::HIGH_STATUS);
                            else if (Remote::rc.ch1 < -0.5) RoboticArmSKD::set_status(RoboticArmSKD::lift_state, LIFT_PAD, RoboticArmSKD::LOW_STATUS);
                            else if (Remote::rc.ch0 > 0.5) RoboticArmSKD::set_status(RoboticArmSKD::extend_state, EXTEND_PAD, RoboticArmSKD::HIGH_STATUS);
                            else if (Remote::rc.ch0 < -0.5) RoboticArmSKD::set_status(RoboticArmSKD::extend_state, EXTEND_PAD, RoboticArmSKD::LOW_STATUS);
                            else if (Remote::rc.ch2 > 0.5) RoboticArmSKD::change_status(RoboticArmSKD::door_state, DOOR_PAD);
                            else if (Remote::rc.ch3 > 0.5) RoboticArmSKD::set_clamp_action(RoboticArmSKD::CLAMP_CLAMPED);
                            else if (Remote::rc.ch3 < -0.5) RoboticArmSKD::set_clamp_action(RoboticArmSKD::CLAMP_RELAX);
                            else if (Remote::rc.ch2 < -0.5) RoboticArmSKD::pull_back();
                            break;
                    }
                    break;
                case Remote::S_DOWN:
                    switch (s2_present_state){
                        case Remote::S_UP:
                            break;
                        case Remote::S_MIDDLE:
                            break;
                        case Remote::S_DOWN:
                            break;
                    }
                    break;
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