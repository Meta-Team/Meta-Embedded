//
// Created by liuzikai on 2019-06-25.
// Edited by Qian Chen & Mo Kanya on 2019-07-05
//

#include "user_hero.h"


/// Gimbal Config
float UserH::gimbal_rc_yaw_max_speed = 90;  // [degree/s]
float UserH::gimbal_pc_yaw_sensitivity[3] = {25000, 75000, 125000};  // [Slow, Normal, Fast] [degree/s]

float UserH::gimbal_pc_pitch_sensitivity[3] = {18000, 48000, 60000};   // [Slow, Normal, Fast] [degree/s]
float UserH::gimbal_pitch_min_angle = -10; // down range for pitch [degree]
float UserH::gimbal_pitch_max_angle = 45; //  up range for pitch [degree]

/// Chassis Config
float UserH::chassis_v_left_right = 1100.0f;  // [mm/s]      //TODO:
float UserH::chassis_v_forward = 1600.0f;     // [mm/s]
float UserH::chassis_v_backward = 1600.0f;    // [mm/s]

float UserH::chassis_pc_shift_ratio = 1.5f;  // 150% when Shift is pressed
float UserH::chassis_pc_ctrl_ratio = 0.5;    // 50% when Ctrl is pressed

Remote::key_t UserH::chassis_dodge_switch = Remote::KEY_X;

/// Shoot Config
float UserH::shoot_launch_left_count = 5;
float UserH::shoot_launch_right_count = 999;

float UserH::shoot_launch_speed = 5.0f;
uint16_t UserH::shoot_heat_log[2] = {0,0};
uint16_t UserH::bullet_heat = 0;

float UserH::shoot_badass_duty_cycle = 0.1f;
float UserH::shoot_remote_duty_cycle = 0.11;
float UserH::shoot_common_duty_cycle = 0.13f;             //TODO:

Remote::key_t UserH::shoot_fw_switch = Remote::KEY_Z;
Remote::key_t UserH::shoot_weapon_switch = Remote::KEY_Q;


/// Variables
UserH::UserThread UserH::userThread;
UserH::UserActionThread UserH::userActionThread;
UserH::ClientDataSendingThread UserH::clientDataSendingThread;

void UserH::start(tprio_t user_thd_prio, tprio_t user_action_thd_prio, tprio_t client_data_sending_thd_prio) {
    userThread.start(user_thd_prio);
    userActionThread.start(user_action_thd_prio);
    clientDataSendingThread.start(client_data_sending_thd_prio);

    // Normal speed, 2 lights from right
    set_user_client_speed_light_(2);
}

void UserH::UserThread::main() {
    setName("UserH");
    while (!shouldTerminate()) {

        /*** ---------------------------------- Gimbal --------------------------------- ***/

        if (!InspectorH::remote_failure() && !InspectorH::chassis_failure() && !InspectorH::gimbal_failure()) {

            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {

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

                float yaw_delta = -Remote::mouse.x * (yaw_sensitivity * USER_THREAD_INTERVAL / 1000.0f);
                float pitch_delta = -Remote::mouse.y * (pitch_sensitivity * USER_THREAD_INTERVAL / 1000.0f);

                VAL_CROP(yaw_delta, 1.5, -1.5);
                VAL_CROP(pitch_delta, 1, -1);

                gimbal_yaw_target_angle_ += yaw_delta;
                // mouse.x use right as positive direction, while GimbalLG use CCW (left) as positive direction


                gimbal_pc_pitch_target_angle_ += pitch_delta;
                // mouse.y use down as positive direction, while GimbalLG use CCW (left) as positive direction

                VAL_CROP(gimbal_pc_pitch_target_angle_, gimbal_pitch_max_angle, gimbal_pitch_min_angle);


                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_);

            } else {
                /// Safe Mode
                GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
            }

        } else {  // InspectorH::remote_failure() || InspectorH::chassis_failure() || InspectorH::gimbal_failure()
            /// Safe Mode
            GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
        }


        /*** ---------------------------------- Shoot --------------------------------- ***/

        // Update the heat.
        shoot_heat_log[0] = shoot_heat_log[1];
        shoot_heat_log[1] = Referee::power_heat_data.shooter_heat1;

        if(shoot_heat_log[1] > shoot_heat_log[0] && shoot_heat_log[1] < Referee::game_robot_state.shooter_heat1_cooling_limit + 400) {
            bullet_heat = shoot_heat_log[1] - shoot_heat_log[0];
        }

        if (!InspectorH::remote_failure() && !InspectorH::chassis_failure() && !InspectorH::gimbal_failure()) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) /*||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN)*/) {

                /// Remote - Shoot with Scrolling Wheel

                if (Remote::rc.wheel > 0.5) {  // down
                    if (HeroShootLG::get_friction_wheels_duty_cycle() == 0) {  // force start friction wheels
                        HeroShootLG::set_friction_wheels(shoot_remote_duty_cycle);
                    }
                    HeroShootLG::shoot();
                } else if (Remote::rc.wheel < -0.5) {  // up
                    if (HeroShootLG::get_friction_wheels_duty_cycle() == 0) {  // force start friction wheels
                        HeroShootLG::set_friction_wheels(shoot_remote_duty_cycle);
                    }
                    HeroShootLG::shoot();
                }

                HeroShootLG::set_friction_wheels(shoot_remote_duty_cycle);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                // UserActionThread handles launching and status of friction wheels

            } else {
                /// Safe Mode
                HeroShootLG::force_stop();
                HeroShootLG::set_friction_wheels(0);
            }

        } else {  // InspectorH::remote_failure() || InspectorH::chassis_failure() || InspectorH::gimbal_failure()
            /// Safe Mode
            HeroShootLG::force_stop();
            HeroShootLG::set_friction_wheels(0);
        }

        /*** ---------------------------------- Chassis --------------------------------- ***/

        if (!InspectorH::remote_failure() && !InspectorH::chassis_failure() && !InspectorH::gimbal_failure()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

                /// Remote - Chassis Move + Chassis Follow
                ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                ChassisLG::set_target(Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                                      (Remote::rc.ch3 > 0 ?
                                       Remote::rc.ch3 * chassis_v_forward :
                                       Remote::rc.ch3 * chassis_v_backward)   // Both use up    as positive direction
                );

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {

                /// Remote - Chassis Move + Chassis Dodge
                ChassisLG::set_action(ChassisLG::DODGE_MODE);
                ChassisLG::set_target(Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                                      (Remote::rc.ch3 > 0 ?
                                       Remote::rc.ch3 * chassis_v_forward :
                                       Remote::rc.ch3 * chassis_v_backward)   // Both use up    as positive direction
                );
            } /*else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {
                /// Remote - Chassis Stop (with Vision)
                ChassisLG::set_action(ChassisLG::FORCED_RELAX_MODE);
            }*/ else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                if (ChassisLG::get_action() == ChassisLG::FORCED_RELAX_MODE) {
                    // Enter PC Mode from other mode, re-enable ChassisLG
                    ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                }

                float target_vx, target_vy;

                if (Remote::key.w) target_vy = chassis_v_forward;
                else if (Remote::key.s) target_vy = -chassis_v_backward;
                else target_vy = 0;

                if (Remote::key.d) target_vx = chassis_v_left_right;
                else if (Remote::key.a) target_vx = -chassis_v_left_right;
                else target_vx = 0;

                if (Remote::key.ctrl) {
                    target_vx *= chassis_pc_ctrl_ratio;
                    target_vy *= chassis_pc_ctrl_ratio;
                } else if (Remote::key.shift) {
                    target_vx *= chassis_pc_shift_ratio;
                    target_vy *= chassis_pc_shift_ratio;
                }
                // No need to echo to user since it has been done above

                ChassisLG::set_target(target_vx, target_vy);

            } else {
                /// Safe Mode
                ChassisLG::set_action(ChassisLG::FORCED_RELAX_MODE);
            }

        } else {  // InspectorH::remote_failure() || InspectorH::chassis_failure() || InspectorH::gimbal_failure()
            /// Safe Mode
            ChassisLG::set_action(ChassisLG::FORCED_RELAX_MODE);
        }


        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}

void UserH::set_user_client_speed_light_(int level) {
    Referee::set_client_light(USER_CLIENT_SPEED_LEVEL_3_LIGHT, (level >= 3));
    Referee::set_client_light(USER_CLIENT_SPEED_LEVEL_2_LIGHT, (level >= 2));
    Referee::set_client_light(USER_CLIENT_SPEED_LEVEL_1_LIGHT, (level >= 1));
}

void UserH::UserActionThread::main() {
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

//            eventflags_t mouse_flag = chEvtGetAndClearFlags(&mouse_press_listener);

            /// Shoot
            if (HeroShootLG::get_friction_wheels_duty_cycle() == 0) {  // force start friction wheels
                HeroShootLG::set_friction_wheels(shoot_common_duty_cycle);
            }
            if(bullet_heat + shoot_heat_log[1] < Referee::game_robot_state.shooter_heat1_cooling_limit) {
                HeroShootLG::shoot();
            }
        } else {  // releasing one while pressing another won't result in stopping
            if (events & MOUSE_RELEASE_EVENTMASK) {
                // HeroShootLG::force_stop();
            }
        }

        /// Key Press
        if (events & KEY_PRESS_EVENTMASK) {

            eventflags_t key_flag = chEvtGetAndClearFlags(&key_press_listener);

            /// Chassis
            if (key_flag & (1U << chassis_dodge_switch)) {
                if (ChassisLG::get_action() == ChassisLG::FOLLOW_MODE) {
                    ChassisLG::set_action(ChassisLG::DODGE_MODE);
                } else if (ChassisLG::get_action() == ChassisLG::DODGE_MODE) {
                    ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                }
            }

            /// Shoot
            if (key_flag & (1U << shoot_fw_switch)) {
                if (HeroShootLG::get_friction_wheels_duty_cycle() > 0) {
                    HeroShootLG::set_friction_wheels(0);
                } else {
                    HeroShootLG::set_friction_wheels(shoot_common_duty_cycle);
                }
            }
            if (key_flag & (1U << shoot_weapon_switch)) {
                if(HeroShootLG::get_friction_wheels_duty_cycle() == shoot_common_duty_cycle){
                    HeroShootLG::set_friction_wheels(shoot_badass_duty_cycle);
                } else if (HeroShootLG::get_friction_wheels_duty_cycle() == shoot_badass_duty_cycle){
                    HeroShootLG::set_friction_wheels(shoot_common_duty_cycle);
                }
            }
        }

        // If more event type is added, remember to modify chEvtWaitAny() above

        // Referee client data will be sent by ClientDataSendingThread

    }
}

void UserH::ClientDataSendingThread::main() {
    setName("User_Client");

    bool super_capacitor_light_status_ = false;
    int flash_time = SYSTIME;
    bool fw_light_switch = false;

    while (!shouldTerminate()) {

        /// Shoot
        // 42mm shooter heat
        Referee::set_client_number(USER_CLIENT_REMAINING_HEAT_NUM,
                                   100.0f * (1.0f - (float) Referee::power_heat_data.shooter_heat1 /
                                                    (float) Referee::game_robot_state.shooter_heat1_cooling_limit));
        // Launch system light. Red: Friction wheel disabled. Flash: Friction wheel enabled, loading
        // Green: Friction wheel enabled, loaded.
        if(HeroShootLG::get_friction_wheels_duty_cycle() == 0) {
            Referee::set_client_light(1, false);
        } else if(HeroShootLG::get_friction_wheels_duty_cycle() != 0) {
            if (HeroShootLG::get_loading_status()) {
                Referee::set_client_light(1, true);
            } else if(!HeroShootLG::get_loading_status()) {
                if(SYSTIME - flash_time > 500) {
                    fw_light_switch = !fw_light_switch;
                    flash_time = SYSTIME;
                }
                Referee::set_client_light(1, fw_light_switch);
            }
        }

        /// Super Capacitor
        // TODO: determine feedback interval
        if (WITHIN_RECENT_TIME(SuperCapacitor::last_feedback_time, 1000)) {
//            Referee::set_client_number(USER_CLIENT_ACTUAL_POWER_NUM, SuperCapacitor::feedback.output_power);
            Referee::set_client_number(USER_CLIENT_SUPER_CAPACITOR_VOLTAGE_NUM,
                                       SuperCapacitor::feedback.capacitor_voltage);
            if (SuperCapacitor::feedback.capacitor_voltage > SUPER_CAPACITOR_WARNING_VOLTAGE) {
                super_capacitor_light_status_ = true;
            } else {
                super_capacitor_light_status_ = not super_capacitor_light_status_;  // blink voltage light
            }
        } else {
//            Referee::set_client_number(USER_CLIENT_ACTUAL_POWER_NUM, 0);
            Referee::set_client_number(USER_CLIENT_SUPER_CAPACITOR_VOLTAGE_NUM, 0);
            super_capacitor_light_status_ = false;
        }
        Referee::set_client_light(USER_CLIENT_SUPER_CAPACITOR_STATUS_LIGHT, super_capacitor_light_status_);

        /// Send data
        Referee::request_to_send(Referee::CLIENT);

        sleep(TIME_MS2I(CLIENT_DATA_SENDING_THREAD_INTERVAL));
    }
}