//
// Created by liuzikai on 2019-06-25.
//

#include "user_engineer.h"

/// Chassis Config
float UserE::chassis_v_left_right = 600.0f;  // [mm/s]
float UserE::chassis_v_forward = 1000.0f;     // [mm/s]
float UserE::chassis_v_backward = 1000.0f;    // [mm/s]
float UserE::chassis_w = 90.0f;    // [degree/s]

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
 * Left  Right   Mode
 * ------------------------------------------------------------
 *  UP    UP/MID Safe
 *  UP    DOWN   Sensors & Swithces Test Mode
 *  MID   UP     Remote - Chassis remote controlling
 *  MID   MID    Remote - Elevator remote controlling
 *  MID   DOWN   Remote - Gimbal remote controlling
 *  DOWN  *      PC     - AUTO_ELEVATING
 *  -Others-     Safe
 * ------------------------------------------------------------
 */

void UserE::UserThread::main() {
    setName("UserE");
    while (!shouldTerminate()) {


        /// Sensors and Switches Test
        if (Remote::rc.s1 == Remote::S_UP && Remote::rc.s2 == Remote::S_DOWN) {
            if (palReadPad(FF_SWITCH_PAD, FFL_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS) {
                LED::red_toggle();
                LOG("LEFT SWITCH ON!");
            }
            if (palReadPad(FF_SWITCH_PAD, FFR_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS) {
                LED::red_toggle();
                LOG("RIGHT  SWITCH ON!");
            }
            else { LED::red_off(); }
        }


        /// Elevator and Chassis

        if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

            EngineerElevatorLG::set_test_mode(true);

            EngineerChassisSKD::enable(true);
            EngineerElevatorLG::elevator_enable(true);

            /// Remote chassis, left FBLR, right turn
            EngineerChassisSKD::set_velocity(
                    Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                    (Remote::rc.ch3 > 0 ?
                     Remote::rc.ch3 * chassis_v_forward :
                     Remote::rc.ch3 * chassis_v_backward),  // Both use up    as positive direction
                    -Remote::rc.ch0 *
                    chassis_w             // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction
            );
        } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {
            /// Remote elevator, left aided motor, right elevator

            EngineerElevatorLG::set_test_mode(true);

            EngineerChassisSKD::enable(true);
            EngineerElevatorLG::elevator_enable(true);

            // right elevator
            if (Remote::rc.ch1 > 0.5 || Remote::rc.ch1 < -0.5)
                EngineerElevatorLG::set_elevator_height(
                        EngineerElevatorLG::get_elevator_height() + Remote::rc.ch1 * 0.5);
            else
                EngineerElevatorSKD::set_target_height(EngineerElevatorLG::get_elevator_height());

            // left aided motor
            if (Remote::rc.ch3 > 0.2 || Remote::rc.ch3 < -0.2)
                EngineerElevatorLG::set_aided_motor_velocity(Remote::rc.ch3 * 0.7 * ENGINEER_AIDED_MOTOR_VELOCITY);
            else
                EngineerElevatorLG::set_aided_motor_velocity(0);

        }
        else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {
            EngineerElevatorLG::set_test_mode(true);

            EngineerChassisSKD::enable(true);
            EngineerElevatorLG::elevator_enable(true);

            if (Remote::rc.ch1 > 0.5 || Remote::rc.ch1 < -0.5){
                EngineerChassisSKD::pivot_turn(CHASSIS_WIDTH / 2, SIGN(Remote::rc.ch1) * CHASSIS_LENGTH / 2, -0.1 * Remote::rc.ch0 * chassis_w);
            } else if (Remote::rc.ch3 > 0.5 || Remote::rc.ch3 < -0.5)
                EngineerChassisSKD::pivot_turn(- CHASSIS_WIDTH / 2, SIGN(Remote::rc.ch3) * CHASSIS_LENGTH / 2, -0.1 * Remote::rc.ch2 * chassis_w);
            else
                EngineerChassisSKD::set_velocity(0,0,0);
        }else if (Remote::rc.s1 == Remote::S_DOWN) {

            EngineerElevatorLG::set_test_mode(false);

            EngineerChassisSKD::enable(true);
            EngineerElevatorLG::elevator_enable(true);

            /// PC control

            if (EngineerElevatorLG::get_current_state() == EngineerElevatorLG::STOP ||
                EngineerElevatorLG::get_current_state() == EngineerElevatorLG::GIVING_BULLET) {

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

                /// Chassis WSQE, AD, ctrl, shift
                float target_vx, target_vy, target_w;

                EngineerGimbalIF::set_target_angle(EngineerGimbalIF::get_target_angle(EngineerGimbalIF::YAW)
                                                   + Remote::mouse.x * 500000 * USER_THREAD_INTERVAL / 1000,
                                                   EngineerGimbalIF::get_target_angle(EngineerGimbalIF::PIT)
                                                   + Remote::mouse.y * 500000 * USER_THREAD_INTERVAL / 1000);
                if (Remote::key.w) {
                    target_vy = chassis_v_forward;
                } else if (Remote::key.s) target_vy = -chassis_v_backward;
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
                    target_w *= chassis_pc_ctrl_ratio;

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
            }

        } else {
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
                if (EngineerGimbalIF::get_target_angle(EngineerGimbalIF::YAW) != 108.0f
                || EngineerGimbalIF::get_target_angle(EngineerGimbalIF::PIT) != 62.0f )
                    EngineerGimbalIF::set_target_angle(108.0f, 62.0f);
                else EngineerGimbalIF::set_target_angle(288.0f, 110.0f);
            } else if (key_flag & (1U << Remote::KEY_V)) {
                EngineerElevatorLG::give_bullet();
                EngineerInterface::change_door();
            }

            if (key_flag & (1u << Remote::KEY_R)) {
                EngineerElevatorLG::set_elevate_dir(true);
            }

            if (key_flag & (1U << Remote::KEY_F)) {
                EngineerElevatorLG::set_elevate_dir(false);
            }

            if ((key_flag & (1U << Remote::KEY_G)) &&
                (key_flag & (1U << Remote::KEY_SHIFT)) &&
                (key_flag & (1U << Remote::KEY_CTRL))) {
                EngineerElevatorLG::change_auto_status();
            }

            if (key_flag & (1U << Remote::KEY_Z)) {
                //RoboticArmSKD::change_extend();
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