//
// Created by Kerui Zhu on 7/19/2019.
//

#include "user_aerial.h"

/// Gimbal Config
float UserA::gimbal_rc_yaw_max_speed = 90;  // [degree/s]
float UserA::gimbal_pc_yaw_sensitivity[3] = {50000, 100000, 150000};  // [Slow, Normal, Fast] [degree/s]

float UserA::gimbal_pc_pitch_sensitivity[3] = {20000, 50000, 60000};   // [Slow, Normal, Fast] [degree/s]
float UserA::gimbal_pitch_min_angle = -10; // down range for pitch [degree]
float UserA::gimbal_pitch_max_angle = 45; //  up range for pitch [degree]
/// Shoot Config
float UserA::shoot_launch_left_count = 5;
float UserA::shoot_launch_right_count = 999;

float UserA::shoot_launch_speed = 15.0f;

float UserA::shoot_common_duty_cycle = 0.6;
float UserA::shoot_debug_duty_cycle = 0.1;

Remote::key_t UserA::shoot_fw_switch = Remote::KEY_Z;



/// Variables
float UserA::gimbal_yaw_target_angle_ = 0;
float UserA::gimbal_pc_pitch_target_angle_ = 0;
UserA::UserThread UserA::userThread;
UserA::UserActionThread UserA::userActionThread;
UserA::ClientDataSendingThread UserA::clientDataSendingThread;

void UserA::start(tprio_t user_thd_prio, tprio_t user_action_thd_prio, tprio_t client_data_sending_thd_prio) {
    userThread.start(user_thd_prio);
    userActionThread.start(user_action_thd_prio);
    clientDataSendingThread.start(client_data_sending_thd_prio);

    // Normal speed, 2 lights from right
    set_user_client_speed_light_(2);
}

void UserA::UserThread::main() {
    setName("UserI");
    while (!shouldTerminate()) {

        /*** ---------------------------------- Gimbal --------------------------------- ***/

        if (!InspectorA::remote_failure() /*&& !InspectorI::chassis_failure()*/ && !InspectorA::gimbal_failure()) {

            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN)) {

                /// Remote - Yaw + Pitch

                GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);


                gimbal_yaw_target_angle_ +=
                        -Remote::rc.ch0 * (gimbal_rc_yaw_max_speed * USER_THREAD_INTERVAL / 1000.0f);
                // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction

                float pitch_target;
                if (Remote::rc.ch1 > 0) pitch_target = Remote::rc.ch1 * gimbal_pitch_max_angle;
                else pitch_target = -Remote::rc.ch1 * gimbal_pitch_min_angle;  // GIMBAL_PITCH_MIN_ANGLE is negative
                // ch1 use up as positive direction, while GimbalLG also use up as positive direction


                GimbalLG::set_target(gimbal_yaw_target_angle_, pitch_target);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);

                float yaw_sensitivity, pitch_sensitivity;
                if (Remote::key.ctrl) {
                    yaw_sensitivity = gimbal_pc_yaw_sensitivity[0];
                    pitch_sensitivity = gimbal_pc_pitch_sensitivity[0];

                    // Slow speed, 3 lights from right
                    set_user_client_speed_light_(1);
                } else if (Remote::key.shift) {
                    yaw_sensitivity = gimbal_pc_yaw_sensitivity[2];
                    pitch_sensitivity = gimbal_pc_pitch_sensitivity[2];

                    // High speed, 1 light from right
                    set_user_client_speed_light_(3);
                } else {
                    yaw_sensitivity = gimbal_pc_yaw_sensitivity[1];
                    pitch_sensitivity = gimbal_pc_pitch_sensitivity[1];

                    // Normal speed, 2 lights from right
                    set_user_client_speed_light_(2);
                }
                // Referee client data will be sent by ClientDataSendingThread

                gimbal_yaw_target_angle_ += -Remote::mouse.x * (yaw_sensitivity * USER_THREAD_INTERVAL / 1000.0f);
                // mouse.x use right as positive direction, while GimbalLG use CCW (left) as positive direction


                gimbal_pc_pitch_target_angle_ +=
                        -Remote::mouse.y * (pitch_sensitivity * USER_THREAD_INTERVAL / 1000.0f);
                // mouse.y use down as positive direction, while GimbalLG use CCW (left) as positive direction

                VAL_CROP(gimbal_pc_pitch_target_angle_, gimbal_pitch_max_angle, gimbal_pitch_min_angle);


                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_);

                if (Remote::key.g){
                    Referee::sentry_guiding_direction_s.direction_mask = 0;
                    if (Remote::key.a) Referee::sentry_guiding_direction_s.direction_mask |= 1U << 1U;
                    if (Remote::key.w) Referee::sentry_guiding_direction_s.direction_mask |= 1U << 2U;
                    if (Remote::key.s) Referee::sentry_guiding_direction_s.direction_mask |= 1U << 3U;
                    if (Remote::key.d) Referee::sentry_guiding_direction_s.direction_mask |= 1U << 4U;
                }

            } else {
                /// Safe Mode
                GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
            }

        } else {  // InspectorI::remote_failure() || InspectorI::chassis_failure() || InspectorI::gimbal_failure()
            /// Safe Mode
            GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
        }


        /*** ---------------------------------- Shoot --------------------------------- ***/

        if (!InspectorA::remote_failure() /*&& !InspectorI::chassis_failure()*/ && !InspectorA::gimbal_failure()) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN)) {

                /// Remote - Shoot with Scrolling Wheel

                if (Remote::rc.wheel > 0.5) {  // down
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(shoot_launch_left_count, shoot_launch_speed);
                    }
                } else if (Remote::rc.wheel < -0.5) {  // up
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(shoot_launch_right_count, shoot_launch_speed);
                    }
                } else {
                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
                        ShootLG::stop();
                    }
                }

                if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {
                    ShootLG::set_friction_wheels(shoot_debug_duty_cycle);
                } else {
                    ShootLG::set_friction_wheels(shoot_common_duty_cycle);
                }


            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                // UserActionThread handles launching and status of friction wheels

            } else {
                /// Safe Mode
                ShootLG::stop();
                ShootLG::set_friction_wheels(0);
            }

        } else {  // InspectorI::remote_failure() || InspectorI::chassis_failure() || InspectorI::gimbal_failure()
            /// Safe Mode
            ShootLG::stop();
            ShootLG::set_friction_wheels(0);
        }


        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}

void UserA::set_user_client_speed_light_(int level) {
    Referee::set_client_light(USER_CLIENT_SPEED_LEVEL_3_LIGHT, (level >= 3));
    Referee::set_client_light(USER_CLIENT_SPEED_LEVEL_2_LIGHT, (level >= 2));
    Referee::set_client_light(USER_CLIENT_SPEED_LEVEL_1_LIGHT, (level >= 1));
}

void UserA::UserActionThread::main() {
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

            /// Shoot
            if (ShootLG::get_friction_wheels_duty_cycle() == 0) {  // force start friction wheels
                ShootLG::set_friction_wheels(shoot_common_duty_cycle);
            }
            if (mouse_flag & (1U << Remote::MOUSE_LEFT)) {
                ShootLG::shoot(shoot_launch_left_count, shoot_launch_speed);
            } else if (mouse_flag & (1U << Remote::MOUSE_RIGHT)) {
                ShootLG::shoot(shoot_launch_right_count, shoot_launch_speed);
            } 
        } else {  // releasing one while pressing another won't result in stopping
            if (events & MOUSE_RELEASE_EVENTMASK) {
                ShootLG::stop();
            }
        }

        /// Key Press
        if (events & KEY_PRESS_EVENTMASK) {

            eventflags_t key_flag = chEvtGetAndClearFlags(&key_press_listener);

            if (key_flag & (1U << Remote::KEY_Q)) {
                gimbal_yaw_target_angle_ += 90.0f;
                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_);
            } else if (key_flag & (1U << Remote::KEY_E)) {
                gimbal_yaw_target_angle_ -= 90.0f;
                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_);
            }

            /// Shoot
            if (key_flag & (1U << shoot_fw_switch)) {
                if (ShootLG::get_friction_wheels_duty_cycle() > 0) {
                    ShootLG::set_friction_wheels(0);
                } else {
                    ShootLG::set_friction_wheels(shoot_common_duty_cycle);
                }
            }
        }

        // If more event type is added, remember to modify chEvtWaitAny() above

        // Referee client data will be sent by ClientDataSendingThread

    }
}

void UserA::ClientDataSendingThread::main() {
    setName("User_Client");
    while (!shouldTerminate()) {
        /// Send data
        Referee::request_to_send(Referee::CLIENT);
        sleep(TIME_MS2I(CLIENT_DATA_SENDING_THREAD_INTERVAL));
    }
}