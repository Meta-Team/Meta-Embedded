//
// Created by Kerui Zhu on 7/19/2019.
//

#include "user_aerial.h"

/// Gimbal Config
float UserA::gimbal_rc_yaw_max_speed = 200;  // [degree/s]
float UserA::gimbal_pc_yaw_sensitivity[3] = {50000, 100000, 150000};  // [Slow, Normal, Fast] [degree/s]

float UserA::gimbal_rc_pitch_max_speed = 60;  // [degree/s]
float UserA::gimbal_pc_pitch_sensitivity[3] = {30000, 70000, 90000};   // [Slow, Normal, Fast] [degree/s]

float UserA::gimbal_yaw_min_angle = -200; // left range for yaw [degree]
float UserA::gimbal_yaw_max_angle = 200; // right range for yaw [degree]
float UserA::gimbal_pitch_min_angle = -60; // down range for pitch [degree]
float UserA::gimbal_pitch_max_angle = 0; //  up range for pitch [degree]

/// Shoot Config
bool UserA::shoot_power_on = true;
float UserA::shoot_launch_left_count = 999;
float UserA::shoot_launch_right_count = 999;

float UserA::shoot_launch_speed = 17.0f;

float UserA::shoot_common_duty_cycle = 0.9;
float UserA::shoot_debug_duty_cycle = 0.05;
float UserA::shoot_full_power_duty_cycle = 0.9;

Remote::key_t UserA::shoot_fw_switch = Remote::KEY_Z;

/// Variables
float UserA::gimbal_yaw_target_angle_ = 0;
float UserA::gimbal_pitch_target_angle_ = 0;
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

            if (Remote::rc.s1 == Remote::S_MIDDLE || Remote::rc.s1 == Remote::S_DOWN) {

                // FIXME: now use SENTRY_MODE
                GimbalLG::set_action(GimbalLG::SENTRY_MODE);

                /*float orig_yaw_target_angle = gimbal_yaw_target_angle_;
                float orig_pitch_target_angle = gimbal_pitch_target_angle_;*/

                if (Remote::rc.s1 == Remote::S_MIDDLE) {

                    /// Remote - Yaw + Pitch

                    gimbal_yaw_target_angle_ +=
                            -Remote::rc.ch0 * (gimbal_rc_yaw_max_speed * USER_THREAD_INTERVAL / 1000.0f);
                    // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction

                    gimbal_pitch_target_angle_ +=
                            Remote::rc.ch1 * (gimbal_rc_pitch_max_speed * USER_THREAD_INTERVAL / 1000.0f);
                    // ch1 use up as positive direction, while GimbalLG also use up as positive direction


                    GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pitch_target_angle_);

                } else if (Remote::rc.s1 == Remote::S_DOWN) {

                    /// PC control mode

                    float yaw_sensitivity, pitch_sensitivity;
                    if (Remote::key.ctrl) {
                        yaw_sensitivity = gimbal_pc_yaw_sensitivity[0];
                        pitch_sensitivity = gimbal_pc_pitch_sensitivity[0];

                        // Slow speed, 1 lights from right
                        set_user_client_speed_light_(1);
                    } else if (Remote::key.shift) {
                        yaw_sensitivity = gimbal_pc_yaw_sensitivity[2];
                        pitch_sensitivity = gimbal_pc_pitch_sensitivity[2];

                        // High speed, 3 light from right
                        set_user_client_speed_light_(3);
                    } else {
                        yaw_sensitivity = gimbal_pc_yaw_sensitivity[1];
                        pitch_sensitivity = gimbal_pc_pitch_sensitivity[1];

                        // Normal speed, 2 lights from right
                        set_user_client_speed_light_(2);
                    }
                    // Referee client data will be sent by ClientDataSendingThread

                    if (Remote::key.g) {
                        Referee::sentry_guiding_direction_s.direction_mask = 0;
                        if (Remote::key.a) Referee::sentry_guiding_direction_s.direction_mask |= 1U << 1U;
                        if (Remote::key.w) Referee::sentry_guiding_direction_s.direction_mask |= 1U << 2U;
                        if (Remote::key.s) Referee::sentry_guiding_direction_s.direction_mask |= 1U << 3U;
                        if (Remote::key.d) Referee::sentry_guiding_direction_s.direction_mask |= 1U << 4U;
                    }

                    /// Perform increment
                    gimbal_yaw_target_angle_ += -Remote::mouse.x * (yaw_sensitivity * USER_THREAD_INTERVAL / 1000.0f);
                    // mouse.x use right as positive direction, while GimbalLG use CCW (left) as positive direction

                    gimbal_pitch_target_angle_ +=
                            -Remote::mouse.y * (pitch_sensitivity * USER_THREAD_INTERVAL / 1000.0f);
                    // mouse.y use down as positive direction, while GimbalLG use CCW (left) as positive direction


                }


                /// Perform clips

                /*float yaw_current_angle = GimbalLG::get_relative_angle(GimbalLG::YAW);
                float pitch_current_angle = GimbalLG::get_relative_angle(GimbalLG::PITCH);

                LOG("Pitch from %f to %f, cur = %f", orig_pitch_target_angle, gimbal_pitch_target_angle_, pitch_current_angle);

                if ((gimbal_yaw_target_angle_ < orig_yaw_target_angle &&  // to decrease, and
                     yaw_current_angle < GIMBAL_YAW_MIN_ANGLE + GIMBAL_LIMIT_ANGLE_TOLORANCE  // no enough space
                    ) ||  // or
                    (gimbal_yaw_target_angle_ > orig_yaw_target_angle &&  // to increase, and
                     yaw_current_angle > GIMBAL_YAW_MAX_ANGLE - GIMBAL_LIMIT_ANGLE_TOLORANCE  // no enough space
                    )) {

                    LOG("Yaw give up from %f to %f", orig_yaw_target_angle, gimbal_yaw_target_angle_);
                    gimbal_yaw_target_angle_ = orig_yaw_target_angle;  // give up change

                }

                if ((gimbal_pitch_target_angle_ < orig_pitch_target_angle &&  // to decrease, and
                     pitch_current_angle < GIMBAL_PITCH_MIN_ANGLE + GIMBAL_LIMIT_ANGLE_TOLORANCE  // no enough space
                    ) ||  // or
                    (gimbal_pitch_target_angle_ > orig_pitch_target_angle &&  // to increase, and
                     pitch_current_angle > GIMBAL_PITCH_MAX_ANGLE - GIMBAL_LIMIT_ANGLE_TOLORANCE  // no enough space
                    )) {

                    LOG("Pitch give up from %f to %f", orig_pitch_target_angle, gimbal_pitch_target_angle_);

                    gimbal_pitch_target_angle_ = orig_pitch_target_angle;  // give up change
                }*/

                VAL_CROP(gimbal_yaw_target_angle_, gimbal_yaw_max_angle, gimbal_yaw_min_angle);
                VAL_CROP(gimbal_pitch_target_angle_, gimbal_pitch_max_angle, gimbal_pitch_min_angle);

                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pitch_target_angle_);

            } else {
                /// Safe Mode
                GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
            }

        } else {  // InspectorI::remote_failure() || InspectorI::chassis_failure() || InspectorI::gimbal_failure()
            /// Safe Mode
            GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
        }


        /*** ---------------------------------- Shoot --------------------------------- ***/

        if (!shoot_power_on){
            if (Referee::robot_state.mains_power_shooter_output == 1){
                float pre_duty = ShootLG::get_shoot_speed();
                ShootLG::set_shoot_speed(0);
                sleep(TIME_MS2I(2000));
                ShootLG::set_shoot_speed(pre_duty);
                LOG("POWER ON AGAIN");
            }
        }
        shoot_power_on = (Referee::robot_state.mains_power_shooter_output == 1);

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
                    ShootLG::set_shoot_speed(shoot_debug_duty_cycle);
                } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {
                    ShootLG::set_shoot_speed(shoot_full_power_duty_cycle);
                } else {
                    ShootLG::set_shoot_speed(shoot_common_duty_cycle);
                }


            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                // UserActionThread handles launching and status of friction wheels

            } else {
                /// Safe Mode
                ShootLG::stop();
                ShootLG::set_shoot_speed(0);
            }

        } else {  // InspectorI::remote_failure() || InspectorI::chassis_failure() || InspectorI::gimbal_failure()
            /// Safe Mode
            ShootLG::stop();
            ShootLG::set_shoot_speed(0);
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
            if (ShootLG::get_shoot_speed() == 0) {  // force start friction wheels
                ShootLG::set_shoot_speed(shoot_common_duty_cycle);
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
                VAL_CROP(gimbal_yaw_target_angle_, gimbal_yaw_max_angle, gimbal_yaw_min_angle);
                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pitch_target_angle_);
            } else if (key_flag & (1U << Remote::KEY_E)) {
                gimbal_yaw_target_angle_ -= 90.0f;
                VAL_CROP(gimbal_yaw_target_angle_, gimbal_yaw_max_angle, gimbal_yaw_min_angle);
                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pitch_target_angle_);
            }

            /// Shoot
            if (key_flag & (1U << shoot_fw_switch)) {
                if (ShootLG::get_shoot_speed() > 0) {
                    ShootLG::set_shoot_speed(0);
                } else {
                    ShootLG::set_shoot_speed(shoot_common_duty_cycle);
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