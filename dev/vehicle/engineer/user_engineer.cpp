//
// Created by liuzikai on 2019-06-25.
//

#include "user_engineer.h"

/// Chassis Config
float UserE::chassis_v_left_right = 300.0f;  // [mm/s]
float UserE::chassis_v_forward = 300.0f;     // [mm/s]
float UserE::chassis_v_backward = 300.0f;    // [mm/s]
float UserE::chassis_w = 150.0f;    // [degree/s]

float UserE::chassis_pc_shift_ratio = 1.5f;  // 150% when Shift is pressed
float UserE::chassis_pc_ctrl_ratio = 0.2;    // 20% when Ctrl is pressed

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
 *  UP    *     Safe
 *  MID   UP    Remote - Chassis remote controlling
 *  MID   MID   Remote - Elevator remote controlling
 *  MID   DOWN  Remote - Gimbal remote controlling
 *  DOWN  UP    Safe
 *  DOWN  MID   PC     - USER_CONTROL ELEVATING
 *  DOWN  DOWN  PC     - AUTO_ELEVATING
 *  -Others-    Safe
 * ------------------------------------------------------------
 */

void UserE::UserThread::main() {
    setName("UserE");
    while (!shouldTerminate()) {

        ///Gimbal
        if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {
            /// Remote Control
            gimbal_pc_yaw_target_angle_ = (Remote::rc.ch0 / 2 + 0.5) * EngineerGimbalIF::MAX_ANGLE;
            gimbal_pc_pitch_target_angle_ = (Remote::rc.ch1 / 2 + 0.5) * EngineerGimbalIF::MAX_ANGLE;
        }

        EngineerGimbalIF::set_target_angle(gimbal_pc_yaw_target_angle_, gimbal_pc_pitch_target_angle_);

        /// Elevator and Chassis

        EngineerChassisSKD::enable(true);
        EngineerElevatorLG::elevator_enable(true);

        if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {
            /// Remote chassis, left FBLR, right turn
            EngineerChassisSKD::set_velocity(
                    Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                    (Remote::rc.ch3 > 0 ?
                     Remote::rc.ch3 * chassis_v_forward :
                     Remote::rc.ch3 * chassis_v_backward),  // Both use up    as positive direction
                    -Remote::rc.ch0 * chassis_w             // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction
            );
        } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {
            /// Remote elevator, left aided motor, right elevator

            EngineerChassisSKD::enable(true);
            EngineerElevatorLG::elevator_enable(true);

            // right elevator
            if (Remote::rc.ch1 > 0.5 || Remote::rc.ch1 < -0.5)
                EngineerElevatorLG::set_elevator_height(EngineerElevatorLG::get_elevator_height() + Remote::rc.ch1 * 0.5);
            else
                EngineerElevatorSKD::set_target_height(EngineerElevatorLG::get_elevator_height());

            // left aided motor
            if (Remote::rc.ch3 > 0.2 || Remote::rc.ch3 < -0.2)
                EngineerElevatorLG::set_aided_motor_velocity(Remote::rc.ch3 * 0.7 * ENGINEER_AIDED_MOTOR_VELOCITY);
            else
                EngineerElevatorLG::set_aided_motor_velocity(0);

        } else if (Remote::rc.s1 == Remote::S_DOWN && Remote::rc.s2 != Remote::S_UP) {

            EngineerChassisSKD::enable(true);
            EngineerElevatorLG::elevator_enable(true);

            /// PC control

            /**
             * PC Key Table:
             * ------------------------------------------------------------
             * Key      Function
             * ------------------------------------------------------------
             * QWES     LEFT, UP, RIGHT, DOWN
             * AD       CCW, CW
             * RF       PAUSE + UP-STAIR, DOWN-STAIR
             * CTRL     SLOW MOVEMENT
             * SHIFT    FAST MOVEMENT
             * V        OPEN/CLOSE DOOR
             * C        TURN AROUND
             * MOUSE_L  ROBOTIC_ARM NEXT STEP
             * MOUSE_R  ROBOTIC_ARM PREV STEP
             * ------------------------------------------------------------
             */

            EngineerElevatorLG::set_auto_elevating(Remote::rc.s2 == Remote::S_DOWN);

            /// Chassis WSQE, AD, ctrl, shift
            float target_vx, target_vy, target_w;

            if (Remote::key.w) {
                target_vy = chassis_v_forward;
                LOG("W");
            }
            else if (Remote::key.s) target_vy = -chassis_v_backward;
            else target_vy = 0;

            if (Remote::key.e) target_vx = chassis_v_left_right;
            else if (Remote::key.q) target_vx = -chassis_v_left_right;
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

        } else{
            /// Safe mode
            EngineerChassisSKD::enable(false);
            EngineerElevatorLG::elevator_enable(false);
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

        /// Mouse Press
        if (events & MOUSE_PRESS_EVENTMASK) {
            eventflags_t mouse_flag = chEvtGetAndClearFlags(&mouse_press_listener);
            if (mouse_flag & (1U << Remote::MOUSE_LEFT)) {
                RoboticArmSKD::next_step();
            } else if (mouse_flag & (1U << Remote::MOUSE_RIGHT)) {
                RoboticArmSKD::prev_step();
            }
        }
        
        /// Key Press
        if (events & KEY_PRESS_EVENTMASK) {

            eventflags_t key_flag = chEvtGetAndClearFlags(&key_press_listener);

            /// Robotic Arm

            if (key_flag & (1U << Remote::KEY_C)) {
                EngineerGimbalIF::set_target_angle(((int)(EngineerGimbalIF::get_target_angle(EngineerGimbalIF::YAW) + 180.0f)) % 360, 0);
            }

            else if (key_flag & (1U << Remote::KEY_V)) {
                RoboticArmSKD::change_door();
                EngineerElevatorLG::give_bullet();
            }

            if (key_flag & (1u << Remote::KEY_R)){
                EngineerElevatorLG::going_up();
            }

            if (key_flag & (1U << Remote::KEY_F)){
                EngineerElevatorLG::going_down();
            }
        }

        // If more event type is added, remember to modify chEvtWaitAny() above
        // Referee client data will be sent by ClientDataSendingThread

    }
}

void UserE::ClientDataSendingThread::main() {

    setName("User_Client");

    while (!shouldTerminate()) {

        /// Send data
        Referee::request_to_send(Referee::CLIENT);

        sleep(TIME_MS2I(CLIENT_DATA_SENDING_THREAD_INTERVAL));
    }
}