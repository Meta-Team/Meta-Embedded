//
// Created by liuzikai on 2019-06-25.
//

#include "user_infantry.h"


/// Gimbal Config
float UserI::gimbal_rc_yaw_max_speed = 180;  // [degree/s]
float UserI::gimbal_pc_yaw_sensitivity[3] = {100000, 200000, 300000};  // [Slow, Normal, Fast] [degree/s]

float UserI::gimbal_pc_pitch_sensitivity[3] = {20000, 50000, 60000};   // [Slow, Normal, Fast] [degree/s]
float UserI::gimbal_pitch_min_angle = -30; // down range for pitch [degree]
float UserI::gimbal_pitch_max_angle = 10; //  up range for pitch [degree]

/// Chassis Config
float UserI::base_power = 40.0f;
float UserI::base_v_forward = 1500.0f;
float UserI::chassis_v_left_right = 500.0f;   // [mm/s]
float UserI::chassis_v_forward = 1500.0f;     // [mm/s]
float UserI::chassis_v_backward = 1500.0f;    // [mm/s]

float UserI::chassis_pc_shift_ratio = 1.5f;  // 150% when Shift is pressed
float UserI::chassis_pc_ctrl_ratio = 0.5f;    // 50% when Ctrl is pressed

Remote::key_t UserI::chassis_dodge_switch = Remote::KEY_X;


/// Shoot Config
float UserI::shoot_launch_left_count = 999;
float UserI::shoot_launch_right_count = 5;

float UserI::shoot_launch_speed = 4.0f;   //Feed rate
float UserI::shoot_common_duty_cycle = 0.40;   //Init speed

UserI::shoot_mode_type UserI::shoot_mode = burst;

Remote::key_t UserI::shoot_fw_switch = Remote::KEY_Z;


/// Variables
float UserI::gimbal_yaw_target_angle_ = 0;
float UserI::gimbal_pc_pitch_target_angle_ = 0;
UserI::UserThread UserI::userThread;
UserI::UserActionThread UserI::userActionThread;
UserI::ClientDataSendingThread UserI::clientDataSendingThread;

bool UserI::left_mouse_pressed = false;
bool UserI::right_mouse_pressed = false;

void UserI::start(tprio_t user_thd_prio, tprio_t user_action_thd_prio, tprio_t client_data_sending_thd_prio) {
    userThread.start(user_thd_prio);
    userActionThread.start(user_action_thd_prio);
    clientDataSendingThread.start(client_data_sending_thd_prio);

}

void UserI::UserThread::main() {
    setName("UserI");
    while (!shouldTerminate()) {

        /*** ---------------------------------- Gimbal --------------------------------- ***/
        if (!InspectorI::remote_failure() && !InspectorI::chassis_failure() && !InspectorI::gimbal_failure()) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {

                /// Remote - Yaw + Pitch

                GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);

                gimbal_yaw_target_angle_ +=
                        -Remote::rc.ch0 * (gimbal_rc_yaw_max_speed * USER_THREAD_INTERVAL / 1000.0f);
                // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction

                float pitch_target;
                if (Remote::rc.ch1 > 0) pitch_target += Remote::rc.ch1 * gimbal_pitch_max_angle * 0.1;
                else
                    pitch_target -=
                            Remote::rc.ch1 * gimbal_pitch_min_angle * 0.1;  // GIMBAL_PITCH_MIN_ANGLE is negative
                // ch1 use up as positive direction, while GimbalLG also use up as positive direction

                VAL_CROP(pitch_target, gimbal_pitch_max_angle, gimbal_pitch_min_angle);
                GimbalLG::set_target(gimbal_yaw_target_angle_, pitch_target);

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                /// Vision - Yaw + Pitch

                GimbalLG::set_action(GimbalLG::VISION_MODE);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);
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

//                ///check if it's necessary to use the auxiliary targeting
//                if (Remote::key.b){
//                    GimbalLG::Auxiliary_ON = (GimbalLG::Auxiliary_ON == 1) ? 0 : 1;
//                }
//
//                if (GimbalLG::Auxiliary_ON) {
//                    if (GimbalLG::should_override_operator(yaw_delta, pitch_delta)) {
//                        yaw_delta = VisionPort::enemy_info.yaw_angle;
//                        pitch_delta = VisionPort::enemy_info.pitch_angle;
//                    }
//                }
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

        } else {  // InspectorI::remote_failure() || InspectorI::chassis_failure() || InspectorI::gimbal_failure()
            /// Safe Mode
            GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
        }


        /*** ---------------------------------- Shoot --------------------------------- ***/

        if (!InspectorI::remote_failure() && !InspectorI::chassis_failure() && !InspectorI::gimbal_failure()) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {

                /// Remote - Shoot with Scrolling Wheel

                ShootLG::set_mode(ShootLG::MANUAL_MODE);

                if (Remote::rc.wheel > 0.5) {  // down
                    if (Referee::power_heat_data.shooter_id1_17mm_cooling_heat <
                        (uint16_t) (Referee::game_robot_state.shooter_id1_17mm_cooling_limit * 0.75)) {
                        if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                            ShootLG::shoot(shoot_launch_left_count, shoot_launch_speed);
                        }
                    } else {
                        ShootLG::stop();
                    }
                } else if (Remote::rc.wheel < -0.5) {  // up
                    if (Referee::power_heat_data.shooter_id1_17mm_cooling_heat <
                        Referee::game_robot_state.shooter_id1_17mm_cooling_limit - 0x0015) {
                        if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                            ShootLG::shoot(shoot_launch_right_count, shoot_launch_speed);
                        }
                    } else {
                        ShootLG::stop();
                    }
                } else {
                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
                        ShootLG::stop();
                    }
                }

                ShootLG::set_friction_wheels(shoot_common_duty_cycle);

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                /// Remote - Vision

                // TODO: move to ShootLG
                if (Remote::rc.wheel > 0.5) {  // down
                    ShootLG::set_mode(ShootLG::MANUAL_MODE);
                    if (Referee::power_heat_data.shooter_id1_17mm_cooling_heat <
                        (uint16_t) (Referee::game_robot_state.shooter_id1_17mm_cooling_limit * 0.75)) {
                        if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                            ShootLG::shoot(shoot_launch_left_count, shoot_launch_speed);
                        }
                    } else {
                        ShootLG::stop();
                    }
                } else if (Remote::rc.wheel < -0.5) {  // up
                    ShootLG::set_mode(ShootLG::VISION_AUTO_MODE);
                } else {
                    ShootLG::set_mode(ShootLG::MANUAL_MODE);
                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
                        ShootLG::stop();
                    }
                }

                ShootLG::set_friction_wheels(shoot_common_duty_cycle);
            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                ShootLG::set_mode(ShootLG::MANUAL_MODE);

                /// Read shoot limit

                if (Referee::game_robot_state.shooter_id1_17mm_cooling_rate >= 40) {
                    shoot_launch_speed = Referee::game_robot_state.shooter_id1_17mm_cooling_rate / 10 * 1.25;
                } else {
                    shoot_launch_speed = Referee::game_robot_state.shooter_id1_17mm_cooling_limit / 25;
                }

                /// Control

                ///switch shoot mode


                if (Remote::key.c) {
                    if (shoot_mode == burst) {
                        shoot_mode = single;
                    } else if (shoot_mode == single) {
                        shoot_mode = triple;
                    } else {
                        shoot_mode = burst;
                    }
                }

                /*switch (shoot_mode) {
                    case burst:
                        if (left_mouse_pressed) {  // down
                            if (Referee::power_heat_data.shooter_id1_17mm_cooling_heat <
                                (uint16_t) (Referee::game_robot_state.shooter_id1_17mm_cooling_limit * 0.75)) {
                                if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                                    ShootLG::shoot(shoot_launch_left_count, shoot_launch_speed);
                                }
                            } else {
                                ShootLG::stop();
                            }
                        }
                            break;

                    case single:
                        if (left_mouse_pressed) {  // down
                            if (Referee::power_heat_data.shooter_id1_17mm_cooling_heat <
                                (uint16_t) (Referee::game_robot_state.shooter_id1_17mm_cooling_limit * 0.75)) {
                                if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                                    ShootLG::shoot(1, shoot_launch_speed);
                                }
                            } else {
                                ShootLG::stop();
                            }
                        }
                            break;

                    case triple:
                        if (left_mouse_pressed) {  // down
                            if (Referee::power_heat_data.shooter_id1_17mm_cooling_heat <
                                (uint16_t) (Referee::game_robot_state.shooter_id1_17mm_cooling_limit * 0.75)) {
                                if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                                    ShootLG::shoot(shoot_launch_right_count, shoot_launch_speed);
                                }
                            } else {
                                ShootLG::stop();
                            }
                        }
                            break;

                            default:
                                if (left_mouse_pressed) {  // down
                                    if (Referee::power_heat_data.shooter_id1_17mm_cooling_heat <
                                        (uint16_t) (Referee::game_robot_state.shooter_id1_17mm_cooling_limit * 0.75)) {
                                        if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                                            ShootLG::shoot(shoot_launch_left_count, shoot_launch_speed);
                                        }
                                    } else {
                                        ShootLG::stop();
                                    }
                                }
                                    break;

                }*/


                if (left_mouse_pressed) {  // down
                    if (Referee::power_heat_data.shooter_id1_17mm_cooling_heat <
                        (uint16_t) (Referee::game_robot_state.shooter_id1_17mm_cooling_limit * 0.75)) {
                        if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                            ShootLG::shoot(shoot_launch_left_count, shoot_launch_speed);
                        }
                    } else {
                        ShootLG::stop();
                    }
                } else if (right_mouse_pressed) {  // up
                    if (Referee::power_heat_data.shooter_id1_17mm_cooling_heat <
                        (uint16_t) (Referee::game_robot_state.shooter_id1_17mm_cooling_limit * 0.75)) {
                        if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                            ShootLG::shoot(shoot_launch_right_count, shoot_launch_speed);
                        }
                    } else {
                        ShootLG::stop();
                    }
                } else if (Remote::key.r) {
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(-1, shoot_launch_speed);
                        ShootLG::shoot(1, shoot_launch_speed);
                    }
                } else {
                    ShootLG::stop();
                }
                // UserActionThread handles launching and status of friction wheels

            } else {
                /// Safe Mode
                ShootLG::force_stop();
                ShootLG::set_friction_wheels(0);
            }

        } else {  // InspectorI::remote_failure() || InspectorI::chassis_failure() || InspectorI::gimbal_failure()
            /// Safe Mode
            ShootLG::stop();
            ShootLG::set_friction_wheels(0);
        }

        /*** ---------------------------------- Chassis --------------------------------- ***/

        if (!InspectorI::remote_failure() && !InspectorI::chassis_failure() && !InspectorI::gimbal_failure()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

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

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                /// Safe for Vision debug
                ChassisLG::set_action(ChassisLG::FORCED_RELAX_MODE);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                /// Read current level information
                chassis_v_forward = Referee::game_robot_state.chassis_power_limit * 0.9 / base_power * base_v_forward;
                chassis_v_backward = chassis_v_forward;

                /// Control

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

        } else {  // InspectorI::remote_failure() || InspectorI::chassis_failure() || InspectorI::gimbal_failure()
            /// Safe Mode
            ChassisLG::set_action(ChassisLG::FORCED_RELAX_MODE);
        }


        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}

void UserI::UserActionThread::main() {
    setName("UserI_Action");

    chEvtRegisterMask(&Remote::s_change_event, &s_change_listener, S_CHANGE_EVENTMASK);
    chEvtRegisterMask(&Remote::mouse_press_event, &mouse_press_listener, MOUSE_PRESS_EVENTMASK);
    chEvtRegisterMask(&Remote::mouse_release_event, &mouse_release_listener, MOUSE_RELEASE_EVENTMASK);
    chEvtRegisterMask(&Remote::key_press_event, &key_press_listener, KEY_PRESS_EVENTMASK);
    chEvtRegisterMask(&Remote::key_release_event, &key_release_listener, KEY_RELEASE_EVENTMASK);

    // FIXME: flags are ORed together among events!!!

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
                left_mouse_pressed = true;
            } else if (mouse_flag & (1U << Remote::MOUSE_RIGHT)) {
                right_mouse_pressed = true;
            }
        } else {
            if (events & MOUSE_RELEASE_EVENTMASK) {
                left_mouse_pressed = false;
                right_mouse_pressed = false;
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

            if (key_flag & (1U << Remote::KEY_Q)) {
                gimbal_yaw_target_angle_ += 90.0f;
                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_);
            } else if (key_flag & (1U << Remote::KEY_E)) {
                gimbal_yaw_target_angle_ -= 90.0f;
                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_);
            }

            /// Shoot
            if (key_flag & (1U << shoot_fw_switch)) {
                if (ABS(ShootLG::get_friction_wheels_duty_cycle()) > 0) {
                    ShootLG::set_friction_wheels(0);
                } else {
                    ShootLG::set_friction_wheels(shoot_common_duty_cycle);
                }
            }

            // TODO: re-arrange variables
            if (key_flag & (1U << Remote::KEY_R)) {
                ShootLG::set_friction_wheels(0.95);
            }
            if (key_flag & (1U << Remote::KEY_F)) {
                ShootLG::set_friction_wheels(0.5);
            }
        }

        // If more event type is added, remember to modify chEvtWaitAny() above

        // Referee client data will be sent by ClientDataSendingThread

    }
}

void UserI::ClientDataSendingThread::main() {
    setName("UserI_Client");

    bool super_capacitor_light_status_ = false;

    while (!shouldTerminate()) {

        /// Shoot
        // 17mm shooter heat

        /// Super Capacitor
        // TODO: determine feedback interval
        if (WITHIN_RECENT_TIME(SuperCapacitor::last_feedback_time, 1000)) {

        } else {

        }

        sleep(TIME_MS2I(CLIENT_DATA_SENDING_THREAD_INTERVAL));
    }
}