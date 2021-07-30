//
// Created by liuzikai on 2019-06-25.
//

#include "user_hero.h"


/// Gimbal Config
float UserH::gimbal_rc_yaw_max_speed = 60;  // [degree/s]
float UserH::gimbal_pc_yaw_sensitivity[3] = {50000, 100000, 150000};  // [Slow, Normal, Fast] [degree/s]

float UserH::gimbal_pc_pitch_sensitivity[3] = {20000, 50000, 60000};   // [Slow, Normal, Fast] [degree/s]
float UserH::gimbal_pitch_min_angle = -25; // down range for pitch [degree]
float UserH::gimbal_pitch_max_angle = 5; //  up range for pitch [degree]

/// Chassis Config
float UserH::base_power = 40.0f;
float UserH::base_v_forward = 1500.0f;
float UserH::chassis_v_left_right = 2000.0f;  // [mm/s]
float UserH::chassis_v_forward = 3000.0f;     // [mm/s]
float UserH::chassis_v_backward = 3000.0f;    // [mm/s]

float UserH::chassis_pc_shift_ratio = 1.5f;  // 150% when Shift is pressed
float UserH::chassis_pc_ctrl_ratio = 0.5;    // 50% when Ctrl is pressed


/// Shoot Config
float UserH::shoot_launch_left_count = 5;
float UserH::shoot_launch_right_count = 999;

float UserH::shoot_feed_rate = 5.0f;   // [bullet/s]
float UserH::shoot_fw_speed[3] = {750, 1200, 2000};  // [Slow, Normal, Fast] [deg/s]


/// Variables
float UserH::gimbal_yaw_target_angle_ = 0.0f;
float UserH::gimbal_pc_pitch_target_angle_ = 0.0f;
float UserH::gimbal_pc_sub_pitch_target_angle_ = 0.0f;
UserH::pitch_separate_mode_t UserH::pitch_separated = IN_ACTIVE;
UserH::UserThread UserH::userThread;
UserH::UserActionThread UserH::userActionThread;

void UserH::start(tprio_t user_thd_prio, tprio_t user_action_thd_prio) {
    userThread.start(user_thd_prio);
    userActionThread.start(user_action_thd_prio);
}

void UserH::UserThread::main() {
    setName("UserH");
    while (!shouldTerminate()) {

        /*** ---------------------------------- Gimbal --------------------------------- ***/
        if (!InspectorH::remote_failure() /*&& !InspectorI::chassis_failure()*/ && !InspectorH::gimbal_failure()) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {

                /// Remote - Yaw + Pitch

                GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);

                gimbal_yaw_target_angle_ +=
                        -Remote::rc.ch0 * (gimbal_rc_yaw_max_speed * USER_THREAD_INTERVAL / 1000.0f);
                // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction

                float pitch_target;
                if (Remote::rc.ch1 > 0) pitch_target += Remote::rc.ch1 * gimbal_pitch_max_angle * 0.1;
                else pitch_target -= Remote::rc.ch1 * gimbal_pitch_min_angle * 0.1;  // GIMBAL_PITCH_MIN_ANGLE is negative
                // ch1 use up as positive direction, while GimbalLG also use up as positive direction

                VAL_CROP(pitch_target, gimbal_pitch_max_angle, gimbal_pitch_min_angle);
                GimbalLG::set_target(gimbal_yaw_target_angle_, pitch_target, 0);

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

//                /// Vision - Change bullet speed with right vertical handle
//
//                /// Vision - Yaw + Pitch
//                GimbalLG::set_action(GimbalLG::VISION_MODE);
//
//                if (Remote::rc.ch1 > 0.5) VisionSKD::set_bullet_speed(VisionSKD::get_bullet_speed() - 0.001f);
//                else if (Remote::rc.ch1 <= -0.5) VisionSKD::set_bullet_speed(VisionSKD::get_bullet_speed() + 0.001f);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

//                if (Remote::key.shift && Remote::key.v) {
//                    VisionSKD::set_bullet_speed(VisionSKD::get_bullet_speed() + 0.001f);
//                } else if (Remote::key.ctrl && Remote::key.v) {
//                    VisionSKD::set_bullet_speed(VisionSKD::get_bullet_speed() - 0.001f);
//                }

                if (Remote::mouse.press_right) {

//                    GimbalLG::set_action(GimbalLG::VISION_MODE);
//
//                    gimbal_yaw_target_angle_ = GimbalLG::get_actual_angle(GimbalLG::YAW);
//                    gimbal_pc_pitch_target_angle_ = GimbalLG::get_actual_angle(GimbalLG::PITCH);

                } else {

                    GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);
                    if (pitch_separated == SEPARATED) {
                        GimbalLG::cal_separate_angle(gimbal_pc_pitch_target_angle_, gimbal_pc_sub_pitch_target_angle_);
                    } else if (pitch_separated == MERGED) {
                        GimbalLG::cal_merge_pitch(gimbal_pc_pitch_target_angle_, gimbal_pc_sub_pitch_target_angle_);
                        if (ABS_IN_RANGE(GimbalLG::get_current_target_angle(GimbalBase::SUB_PITCH), 0.5)) {
                            pitch_separated = IN_ACTIVE;
                            gimbal_pc_sub_pitch_target_angle_ = 0.0f;
                        }
                    } else {
                        float yaw_sensitivity, pitch_sensitivity;
                        if (Remote::key.ctrl) {
                            yaw_sensitivity = gimbal_pc_yaw_sensitivity[0];
                            pitch_sensitivity = gimbal_pc_pitch_sensitivity[0];
                        } else if (Remote::key.shift) {
                            yaw_sensitivity = gimbal_pc_yaw_sensitivity[2];
                            pitch_sensitivity = gimbal_pc_pitch_sensitivity[2];
                        } else {
                            yaw_sensitivity = gimbal_pc_yaw_sensitivity[1];
                            pitch_sensitivity = gimbal_pc_pitch_sensitivity[1];
                        }
                        // Referee client data will be sent by ClientDataSendingThread

                        float yaw_delta = -Remote::mouse.x * (yaw_sensitivity * USER_THREAD_INTERVAL / 1000.0f);
                        float pitch_delta = Remote::mouse.y * (pitch_sensitivity * USER_THREAD_INTERVAL / 1000.0f);

                        VAL_CROP(yaw_delta, 1.5, -1.5);
                        VAL_CROP(pitch_delta, 1, -1);

                        gimbal_yaw_target_angle_ += yaw_delta;
                        // mouse.x use right as positive direction, while GimbalLG use CCW (left) as positive direction

                        gimbal_pc_pitch_target_angle_ += pitch_delta;
                        // mouse.y use down as positive direction, while GimbalLG use CCW (left) as positive direction
                    }

                    VAL_CROP(gimbal_pc_pitch_target_angle_, gimbal_pitch_max_angle, gimbal_pitch_min_angle);

                    GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_, gimbal_pc_sub_pitch_target_angle_);
                }

            } else {
                /// Safe Mode
                GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
            }

        } else {  // InspectorH::remote_failure() || InspectorH::chassis_failure() || InspectorH::gimbal_failure()
            /// Safe Mode
            GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
        }

        /*** ---------------------------------- Shoot --------------------------------- ***/
        if (!InspectorH::remote_failure() /*&& !InspectorH::chassis_failure()*/ && !InspectorH::gimbal_failure()) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {

                /// Remote - Shoot with Scrolling Wheel

                ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);

                if (Remote::rc.wheel > 0.5) {  // down
                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(ShootLG::get_bullet_count_to_heat_limit(), shoot_feed_rate);
                    } else {
                        ShootLG::stop();
                    }
                } else if (Remote::rc.wheel < -0.5) {  // up
                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(999 /* unlimited */, shoot_feed_rate);
                    } else {
                        ShootLG::stop();
                    }
                } else {
                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
                        ShootLG::stop();
                    }
                }

                ShootLG::set_shoot_speed(shoot_fw_speed[1]);

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

//                /// Remote - Vision
//
//                if (Remote::rc.wheel > 0.5) {  // down
//                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
//                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
//                        ShootLG::shoot(ShootLG::get_bullet_count_to_heat_limit(), shoot_feed_rate);
//                    } else {
//                        ShootLG::stop();
//                    }
//                } else if (Remote::rc.wheel < -0.5) {  // up
//                    ShootLG::set_limit_mode(ShootLG::VISION_LIMITED_MODE);
//                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
//                        ShootLG::shoot(ShootLG::get_bullet_count_to_heat_limit(), shoot_feed_rate);
//                    } else {
//                        ShootLG::stop();
//                    }
//                } else {
//                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
//                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
//                        ShootLG::stop();
//                    }
//                }
//
//                ShootLG::set_shoot_speed(shoot_fw_speed[1]);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                if (Remote::mouse.press_left) {

                    if (Remote::mouse.press_right) {
//                        ShootLG::set_limit_mode(ShootLG::VISION_LIMITED_MODE);
                    } else {
                        ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
                    }

                    if (ShootLG::get_shoot_speed() == 0) {  // force start friction wheels
                        ShootLG::set_shoot_speed(shoot_fw_speed[1]);
                    }

                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {  // set target once
                        ShootLG::shoot(ShootLG::get_bullet_count_to_heat_limit(), shoot_feed_rate);
                    }

                } else {

                    ShootLG::stop();

                }

            } else {
                /// Safe Mode
                ShootLG::force_stop();
                ShootLG::set_shoot_speed(0);
            }

        } else {  // InspectorH::remote_failure() || InspectorH::chassis_failure() || InspectorH::gimbal_failure()
            /// Safe Mode
            ShootLG::stop();
            ShootLG::set_shoot_speed(0);
        }

        /*** ---------------------------------- Chassis --------------------------------- ***/
        if (!InspectorH::remote_failure() && !InspectorH::chassis_failure() /*&& !InspectorH::gimbal_failure()*/) {

            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN)) {

                /// Remote - Chassis Move + Chassis Follow
                ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                ChassisLG::set_target(Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                                      (Remote::rc.ch3 > 0 ?
                                       Remote::rc.ch3 * 1000 :
                                       Remote::rc.ch3 * 800)   // Both use up as positive direction
                );

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {

                /// Remote - Chassis Move + Chassis Dodge
                ChassisLG::set_action(ChassisLG::DODGE_MODE);
                ChassisLG::set_target(Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                                      (Remote::rc.ch3 > 0 ?
                                       Remote::rc.ch3 * 1000 :
                                       Remote::rc.ch3 * 800)   // Both use up as positive direction
                );

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                /// Chassis - Movement

                // Read current level information
                chassis_v_forward = Referee::robot_state.chassis_power_limit * 0.9 / base_power * base_v_forward;
                chassis_v_backward = chassis_v_forward;

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

void UserH::UserActionThread::main() {
    setName("User_Action");

//    chEvtRegisterMask(&Remote::s_change_event, &s_change_listener, S_CHANGE_EVENTMASK);
//    chEvtRegisterMask(&Remote::mouse_press_event, &mouse_press_listener, MOUSE_PRESS_EVENTMASK);
//    chEvtRegisterMask(&Remote::mouse_release_event, &mouse_release_listener, MOUSE_RELEASE_EVENTMASK);
    chEvtRegisterMask(&Remote::key_press_event, &key_press_listener, KEY_PRESS_EVENTMASK);
//    chEvtRegisterMask(&Remote::key_release_event, &key_release_listener, KEY_RELEASE_EVENTMASK);

    // FIXME: flags are ORed together among events!!!

    while (!shouldTerminate()) {

        eventmask_t events = chEvtWaitAny(ALL_EVENTS);

        /**
         * NOTICE: use flags instead of direct accessing variable in Remote, since event can be trigger by other key, etc
         */

        /// Key Press
        if (events & KEY_PRESS_EVENTMASK) {

            eventflags_t key_flag = chEvtGetAndClearFlags(&key_press_listener);

            /// Chassis - Dodge Mode Switching
            if (key_flag & (1U << Remote::KEY_X)) {
                if (ChassisLG::get_action() == ChassisLG::FOLLOW_MODE) {
                    ChassisLG::set_action(ChassisLG::DODGE_MODE);
                } else if (ChassisLG::get_action() == ChassisLG::DODGE_MODE) {
                    ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                }
            }

            /// Gimbal
            // Sub-pitch adjustment, only for hero
#ifdef HERO
            if (key_flag & (1U << Remote::KEY_G)) {
                GimbalLG::separate_pitch();
                pitch_separated = SEPARATED;
            } else if (key_flag & (1U << Remote::KEY_B)) {
                pitch_separated = MERGED;
            }
#endif
            if (key_flag & (1U << Remote::KEY_Q)) {
                gimbal_yaw_target_angle_ += 90.0f;
                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_, gimbal_pc_sub_pitch_target_angle_);
            } else if (key_flag & (1U << Remote::KEY_E)) {
                gimbal_yaw_target_angle_ -= 90.0f;
                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_, gimbal_pc_sub_pitch_target_angle_);
            }

            /// Shoot
            if (key_flag & (1U << Remote::KEY_Z)) {
                if (Remote::key.ctrl && Remote::key.shift) {
                    RefereeUILG::reset();
                } else if (Remote::key.ctrl) {
                    ShootLG::set_shoot_speed(shoot_fw_speed[0]);
                } else if (Remote::key.shift) {
                    ShootLG::set_shoot_speed(shoot_fw_speed[2]);
                } else {
                    if (ABS(ShootLG::get_shoot_speed()) > 0) {
                        ShootLG::set_shoot_speed(0);
                    } else {
                        ShootLG::set_shoot_speed(shoot_fw_speed[1]);
                    }
                }
            }

        }

    }
}