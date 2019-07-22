//
// Created by liuzikai on 2019-06-25.
//

#include "user_engineer.h"


/// Gimbal Config
float UserE::gimbal_pc_yaw_sensitivity[3] = {50000, 100000, 150000};  // [Slow, Normal, Fast] [degree/s]
float UserE::gimbal_yaw_min_angle = -120; // down range for yaw [degree]
float UserE::gimbal_yaw_max_angle = 120; //  up range for yaw [degree]

float UserE::gimbal_pc_pitch_sensitivity[3] = {20000, 50000, 60000};   // [Slow, Normal, Fast] [degree/s]
float UserE::gimbal_pitch_min_angle = -10; // down range for pitch [degree]
float UserE::gimbal_pitch_max_angle = 45; //  up range for pitch [degree]

/// Chassis Config
float UserE::chassis_v_left_right = 300.0f;  // [mm/s]
float UserE::chassis_v_forward = 600.0f;     // [mm/s]
float UserE::chassis_v_backward = 600.0f;    // [mm/s]
float UserE::chassis_w = 150.0f;    // [degree/s]

float UserE::chassis_pc_shift_ratio = 1.5f;  // 150% when Shift is pressed
float UserE::chassis_pc_ctrl_ratio = 0.2;    // 20% when Ctrl is pressed

/// Elevator Config
// TODO: apply it
float UserE::aided_motor_v = 600.0f;  // [mm/s]

/// Variables
float UserE::gimbal_pc_yaw_target_angle_ = 0;
float UserE::gimbal_pc_pitch_target_angle_ = 0;
UserE::UserThread UserE::userThread;
UserE::UserActionThread UserE::userActionThread;
UserE::ClientDataSendingThread UserE::clientDataSendingThread;

void UserE::start(tprio_t user_thd_prio, tprio_t user_action_thd_prio, tprio_t client_data_sending_thd_prio) {
    userThread.start(user_thd_prio);
    userActionThread.start(user_action_thd_prio);
    clientDataSendingThread.start(client_data_sending_thd_prio);

    // Normal speed, 2 lights from right
    set_user_client_speed_light_(2);
}


/**
 * Mode Table:
 * ------------------------------------------------------------
 * Left  Right  Mode
 * ------------------------------------------------------------
 *  UP    UP    Safe
 *  UP    MID   Remote - Chassis remote controlling
 *  UP    DOWN  Remote - Elevator remote controlling
 *  MID   UP    Remote - Auto elevating
 *  MID   MID   ***
 *  MID   DOWN  Remote - Robotic Arm test
 *  DOWN  UP    ***
 *  DOWN  MID   ***
 *  DOWN  DOWN  Final PC MODE
 *  -Others-    Safe
 * ------------------------------------------------------------
 */

void UserE::UserThread::main() {
    setName("UserE");
    while (!shouldTerminate()) {

        /// Safe mode
        if (Remote::rc.s1 == Remote::S_UP && Remote::rc.s2 == Remote::S_UP) {
            EngineerChassisSKD::lock();
            EngineerElevatorLG::set_action_lock();
        }

        /// Remote chassis, left FBLR, right turn
        else if (Remote::rc.s1 == Remote::S_UP && Remote::rc.s2 == Remote::S_MIDDLE) {
//            LOG("remote chassis");
            EngineerChassisSKD::unlock();
            EngineerElevatorLG::set_action_lock();
            EngineerChassisSKD::set_velocity(
                    Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                    (Remote::rc.ch3 > 0 ?
                     Remote::rc.ch3 * chassis_v_forward :
                     Remote::rc.ch3 * chassis_v_backward),  // Both use up    as positive direction
                    -Remote::rc.ch0 * chassis_w             // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction
            );
        }

        /// Remote elevator, left aided motor, right elevator
        else if (Remote::rc.s1 == Remote::S_UP && Remote::rc.s2 == Remote::S_DOWN) {
            EngineerChassisSKD::lock();
            EngineerElevatorLG::set_action_free();
            // and disable Robotic arm


            if (Remote::rc.ch1 > 0.5 || Remote::rc.ch1 < -0.5) {
                EngineerElevatorSKD::elevator_enable(true);
                EngineerElevatorSKD::aided_motor_enable(false);
                EngineerElevatorSKD::set_target_height(EngineerElevatorIF::get_current_height() + Remote::rc.ch1 * 2);
            } else if (Remote::rc.ch3 > 0.2 || Remote::rc.ch3 < -0.2) {
                EngineerElevatorSKD::elevator_enable(false);
                EngineerElevatorSKD::aided_motor_enable(true);
                EngineerElevatorSKD::set_aided_motor_velocity(Remote::rc.ch3 * ENGINEER_AIDED_MOTOR_VELOCITY,
                                                              Remote::rc.ch3 * ENGINEER_AIDED_MOTOR_VELOCITY);
            } else if (Remote::rc.ch2 > 0.5) {
                EngineerElevatorSKD::elevator_enable(false);
                EngineerElevatorSKD::aided_motor_enable(true);
                EngineerElevatorSKD::set_aided_motor_velocity(Remote::rc.ch3 * ENGINEER_AIDED_MOTOR_VELOCITY, 0);
            } else if (Remote::rc.ch2 < -0.5) {
                EngineerElevatorSKD::elevator_enable(false);
                EngineerElevatorSKD::aided_motor_enable(true);
                EngineerElevatorSKD::set_aided_motor_velocity(0, Remote::rc.ch3 * ENGINEER_AIDED_MOTOR_VELOCITY);
            } else {
                EngineerElevatorSKD::elevator_enable(false);
                EngineerElevatorSKD::aided_motor_enable(false);
            }
        }


        /// Remote auto elevating. RIGHT.
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

        else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

            if (EngineerElevatorLG::get_action() == EngineerElevatorLG::LOCK) {
                if (Remote::rc.ch1 > 0.5)
                    EngineerElevatorLG::start_going_up();
                else if (Remote::rc.ch2 < -0.5)
                    EngineerElevatorLG::start_going_down();
            } else if (EngineerElevatorLG::get_action() == EngineerElevatorLG::UPWARD) {
                if (Remote::rc.ch2 < -0.5)
                    EngineerElevatorLG::pause_action();
            } else if (EngineerElevatorLG::get_action() == EngineerElevatorLG::DOWNWARD) {
                if (Remote::rc.ch2 < 0.5)
                    EngineerElevatorLG::pause_action();
            } else if (EngineerElevatorLG::get_action() == EngineerElevatorLG::PAUSE) {
                if ((EngineerElevatorLG::get_prev_action() == EngineerElevatorLG::UPWARD && Remote::rc.ch1 > 0.5) ||
                    (EngineerElevatorLG::get_prev_action() == EngineerElevatorLG::DOWNWARD && Remote::rc.ch1 < -0.5))
                    EngineerElevatorLG::continue_action();
                else if ((EngineerElevatorLG::get_prev_action() == EngineerElevatorLG::UPWARD &&
                          Remote::rc.ch1 < -0.5) ||
                         (EngineerElevatorLG::get_prev_action() == EngineerElevatorLG::DOWNWARD &&
                          Remote::rc.ch1 > 0.5))
                    EngineerElevatorLG::quit_action();
            }

        }


            /// Safe
        else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {

            // Set as safe currently
            EngineerChassisSKD::lock();
            EngineerElevatorLG::set_action_lock();
        }

            /// Remote Robotic arm test
        else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

//            if (Remote::rc.ch1 > 0.5) RoboticArmSKD::next_step();
//            else if (Remote::rc.ch1 < -0.5) RoboticArmSKD::prev_step();
//            else if (Remote::rc.ch0 > 0.5) RoboticArmSKD::change_extend();
//            else if (Remote::rc.ch0 < -0.5) RoboticArmSKD::change_door();
        }


            /// Safe
        else if (Remote::rc.s1 == Remote::S_DOWN && Remote::rc.s2 != Remote::S_DOWN) {

            // the same as UP-UP
            EngineerChassisSKD::lock();
            EngineerElevatorLG::set_action_lock();
        }


            /// PC control
        else if (Remote::rc.s1 == Remote::S_DOWN && Remote::rc.s2 == Remote::S_DOWN) {

            /// Chassis WSQE, AD, ctrl, shift

            float target_vx, target_vy, target_w;

            if (Remote::key.w) target_vy = chassis_v_forward;
            else if (Remote::key.s) target_vy = -chassis_v_backward;
            else target_vy = 0;

            if (Remote::key.q) target_vx = chassis_v_left_right;
            else if (Remote::key.e) target_vx = -chassis_v_left_right;
            else target_vx = 0;

            if (Remote::key.a) target_w = chassis_w;
            else if (Remote::key.d) target_w = -chassis_w;
            else target_w = 0;

            if (Remote::key.ctrl) {
                target_vx *= chassis_pc_ctrl_ratio;
                target_vy *= chassis_pc_ctrl_ratio;

                // Slow speed, 1 lights from right
                set_user_client_speed_light_(1);
            } else if (Remote::key.shift) {
                target_vx *= chassis_pc_shift_ratio;
                target_vy *= chassis_pc_shift_ratio;

                // Fast speed, 3 lights from right
                set_user_client_speed_light_(3);
            } else {
                // Normal speed, 2 lights from right
                set_user_client_speed_light_(2);
            }

            EngineerChassisSKD::set_velocity(target_vx, target_vy, target_w);


            /// Elevator, RF
            if (EngineerElevatorLG::get_action() == EngineerElevatorLG::LOCK) {
                if (Remote::key.r) EngineerElevatorLG::start_going_up();
                else if (Remote::key.f) EngineerElevatorLG::start_going_down();
            } else if (EngineerElevatorLG::get_action() == EngineerElevatorLG::UPWARD) {
                if (Remote::key.f) EngineerElevatorLG::pause_action();
            } else if (EngineerElevatorLG::get_action() == EngineerElevatorLG::DOWNWARD) {
                if (Remote::key.r) EngineerElevatorLG::pause_action();
            } else if (EngineerElevatorLG::get_action() == EngineerElevatorLG::PAUSE) {
                if (EngineerElevatorLG::get_prev_action() == EngineerElevatorLG::UPWARD) {
                    if (Remote::key.r) EngineerElevatorLG::continue_action();
                    else if (Remote::key.f) EngineerElevatorLG::quit_action();
                } else if (EngineerElevatorLG::get_prev_action() == EngineerElevatorLG::DOWNWARD) {
                    if (Remote::key.r) EngineerElevatorLG::quit_action();
                    else if (Remote::key.f) EngineerElevatorLG::continue_action();
                }
            }


            /// Robotic Arm

//            if (Remote::key.z) RoboticArmSKD::next_step();
//            else if (Remote::key.x) RoboticArmSKD::prev_step();
//            else if (Remote::key.c) RoboticArmSKD::change_extend();
//            else if (Remote::key.v) RoboticArmSKD::change_door();

            /// supplying bullets

            /// camera


        }

        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}

void UserE::set_user_client_speed_light_(int level) {
    Referee::set_client_light(USER_CLIENT_SPEED_LEVEL_3_LIGHT, (level >= 3));
    Referee::set_client_light(USER_CLIENT_SPEED_LEVEL_2_LIGHT, (level >= 2));
    Referee::set_client_light(USER_CLIENT_SPEED_LEVEL_1_LIGHT, (level >= 1));
}

void UserE::UserActionThread::main() {
    setName("User_Action");

    chEvtRegisterMask(&Remote::s_change_event, &s_change_listener, S_CHANGE_EVENTMASK);
    chEvtRegisterMask(&Remote::mouse_press_event, &mouse_press_listener, MOUSE_PRESS_EVENTMASK);
    chEvtRegisterMask(&Remote::mouse_release_event, &mouse_release_listener, MOUSE_RELEASE_EVENTMASK);
    chEvtRegisterMask(&Remote::key_press_event, &key_press_listener, KEY_PRESS_EVENTMASK);
    chEvtRegisterMask(&Remote::key_release_event, &key_release_listener, KEY_RELEASE_EVENTMASK);

    while (!shouldTerminate()) {

        eventmask_t events = chEvtWaitAny(MOUSE_PRESS_EVENTMASK | MOUSE_RELEASE_EVENTMASK | KEY_PRESS_EVENTMASK);

        /**
         * NOTICE: use flag instead of direct accessing variable in Remote, since event can be trigger by other key, etc
         */

        // If more event type is added, remember to modify chEvtWaitAny() above

        // Referee client data will be sent by ClientDataSendingThread

    }
}

void UserE::ClientDataSendingThread::main() {

    setName("User_Client");

    while (!shouldTerminate()) {

        /// Send data
        Referee::send_data(Referee::CLIENT);

        sleep(TIME_MS2I(CLIENT_DATA_SENDING_THREAD_INTERVAL));
    }
}