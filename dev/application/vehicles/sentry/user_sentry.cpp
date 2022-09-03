//
// Created by Kerui Zhu on 2019-7-16
//

#include "user_sentry.h"

/// Gimbal config
float UserS::gimbal_yaw_min_angle = -160; // left range for yaw [degree]
float UserS::gimbal_yaw_max_angle = 160; // right range for yaw [degree]
float UserS::gimbal_pitch_min_angle = -50; // down range for pitch [degree]
float UserS::gimbal_pitch_max_angle = 0; //  up range for pitch [degree]

/// Chassis config
float UserS::chassis_v = 80.0f;  // [cm/s]

/// Shoot config
float UserS::shoot_feed_rate = 5.0; // [bullets/s]
float UserS::shoot_fw_speed = 42150; // [deg/s] ????

/// Runtime variables
float UserS::gimbal_yaw_target_angle_ = 0;
float UserS::gimbal_pitch_target_angle_ = 0;
bool UserS::ignore_shoot_constraints = false;
/// User thread
UserS::UserThread UserS::userThread;

void UserS::start(tprio_t user_thd_prio) {
    userThread.start(user_thd_prio);
}

/**
 * Mode Table:
 * ------------------------------------------------------------
 * Left  Right  Mode
 * ------------------------------------------------------------
 *  UP    ***   Safe
 *  MID   UP    Remote - Gimbal: Angle Control, Chassis: Manual Mode
 *  MID   MID   Remote - Gimbal: Velocity Control, Chassis: Manual Mode
 *  MID   DOWN  Remote - Gimbal: Scanning Mode, Chassis: Manual Mode
 *  DOWN  UP    Remote - Gimbal: Vision Control, Chassis: Manual Mode
 *  DOWN  MID   Remote - Gimbal: Scanning Mode + Vision Control, Chassis: Shuttle Mode
 *  DOWN  DOWN  Auto   - Gimbal: Scanning Mode + Vision Control, Chassis: Final Auto Mode
 * ------------------------------------------------------------
 */

void UserS::UserThread::main() {
    setName("UserS");
    while (!shouldTerminate()) {
//        /*** ---------------------------------- Shoot --------------------------------- ***/
//
//        if (!shoot_power_on){
//            if (Referee::robot_state.mains_power_shooter_output == 1){
//                float pre_duty = ShootLG::get_shoot_speed();
//                ShootLG::set_shoot_speed(0);
//                sleep(TIME_MS2I(2000));
//                ShootLG::set_shoot_speed(pre_duty);
//                LOG("POWER ON AGAIN");
//            }
//        }
//        shoot_power_on = (Referee::robot_state.mains_power_shooter_output == 1);
//
//        if (!InspectorS::remote_failure() /*&& !InspectorS::chassis_failure()*/ && !InspectorS::gimbal_failure()) {
//            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
//                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {
//
//                /// Remote - Shoot with Scrolling Wheel
//
//                if (Remote::rc.wheel > 0.5) {  // down
//                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
//                        ShootLG::shoot(shoot_launch_left_count, shoot_launch_speed);
//                    }
//                } else if (Remote::rc.wheel < -0.5) {  // up
//                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
//                        ShootLG::shoot(shoot_launch_right_count, shoot_launch_speed);
//                    }
//                } else {
//                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
//                        ShootLG::stop();
//                    }
//                }
//
//                ShootLG::set_shoot_speed(shoot_common_duty_cycle);
//
//            }else if (Remote::rc.s1 == Remote::S_DOWN) {
//
//                /// PC control mode
//
//                if (fire) {
//                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
//                        ShootLG::shoot(shoot_launch_right_count, shoot_launch_speed);
//                    }
//                } else {
//                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
//                        ShootLG::stop();
//                    }
//                }
//
//            } else {
//                /// Safe Mode
//                ShootLG::stop();
//                ShootLG::set_shoot_speed(0);
//            }
//
//        } else {  // InspectorS::remote_failure() || InspectorS::chassis_failure() || InspectorS::gimbal_failure()
//            /// Safe Mode
//            ShootLG::stop();
//            ShootLG::set_shoot_speed(0);
//        }
//// Wu Feiyang: I just grab this stuff from the Infantry
        if (!InspectorS::remote_failure() && !InspectorS::chassis_failure() && !InspectorS::gimbal_failure()) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {

                /// Remote - Shoot with Scrolling Wheel

                ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);

                if (Remote::rc.wheel > 0.5) {  // down
                    ShootLG::set_shoot_speed(shoot_fw_speed);
                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(ignore_shoot_constraints ? 999 : ShootLG::get_bullet_count_to_heat_limit(), shoot_feed_rate);
                    }
                } else if (Remote::rc.wheel < -0.5) {  // up
                    ShootLG::set_shoot_speed(shoot_fw_speed);
                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(999 /* unlimited */, shoot_feed_rate);
                    }
                } else {
                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
                        ShootLG::stop();
                    }
                }

                ShootLG::set_shoot_speed(shoot_fw_speed);

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                /// Remote - Vision

                if (Remote::rc.wheel > 0.5) {  // down
                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(ignore_shoot_constraints ? 999 : ShootLG::get_bullet_count_to_heat_limit(),
                                       shoot_feed_rate);
                    }
                } else if (Remote::rc.wheel < -0.5) {  // up
#if ENABLE_VISION
                    ShootLG::set_limit_mode(ShootLG::VISION_LIMITED_MODE);
#endif
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(ignore_shoot_constraints ? 999 : ShootLG::get_bullet_count_to_heat_limit(),
                                       shoot_feed_rate);
                    }
                } else {
                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
                        ShootLG::stop();
                    }
                }

                ShootLG::set_shoot_speed(shoot_fw_speed);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                if (Remote::mouse.press_left) {
#if ENABLE_REFEREE == TRUE
                    // Read shoot limit
                    if (Referee::robot_state.shooter_id1_17mm_cooling_rate >= 40) {
                        shoot_feed_rate = Referee::robot_state.shooter_id1_17mm_cooling_rate / 10 * 1.25;
                    } else {
                        shoot_feed_rate = Referee::robot_state.shooter_id1_17mm_cooling_limit / 25;
                    }
#if ENABLE_VISION == TRUE
                    none
#endif
#else
                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
#endif
                    if (ShootLG::get_shoot_speed() == 0) {  // force start friction wheels
                        ShootLG::set_shoot_speed(shoot_fw_speed);
                    }

                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {  // set target once
                        ShootLG::shoot(ignore_shoot_constraints ? 999 : ShootLG::get_bullet_count_to_heat_limit(),
                                       shoot_feed_rate);
                    }
                } else {

                    ShootLG::stop();

                }

            } else {
                /// Safe Mode
                ShootLG::force_stop();
                ShootLG::set_shoot_speed(0);
            }

        } else {  // InspectorI::remote_failure() || InspectorI::chassis_failure() || InspectorI::gimbal_failure()
            /// Safe Mode
            ShootLG::stop();
            ShootLG::set_shoot_speed(0);
        }

        /*** ---------------------------------- Gimbal  --------------------------------- ***/

        if (!InspectorS::gimbal_failure() && !InspectorS::remote_failure()) {
            if (Remote::rc.s1 == Remote::S_UP) {
                GimbalLG::set_mode(GimbalLG::FORCED_RELAX_MODE);
            } else if (Remote::rc.s1 == Remote::S_MIDDLE) {
                GimbalLG::set_mode(GimbalLG::CHASSIS_REF_MODE);

                /// Update gimbal targets from remote controller
                gimbal_pitch_target_angle_ += ((Remote::rc.ch1 > 0) ? gimbal_pitch_max_angle : gimbal_pitch_min_angle)
                                                * 0.1f * Remote::rc.ch1;
                gimbal_yaw_target_angle_ += Remote::rc.ch0 * gimbal_yaw_max_angle * 0.1f;

                /// Crop gimbal targets to limited range. Direction remains same for yaw axis.
                VAL_CROP(gimbal_pitch_target_angle_, gimbal_pitch_max_angle, gimbal_pitch_min_angle);

                /// Copy these lines for crop vision target angle.
                float yaw_target_angle_cropped = gimbal_yaw_target_angle_;
                int yaw_rounds = (int)(gimbal_yaw_target_angle_ / 360.0f);
                yaw_target_angle_cropped = yaw_target_angle_cropped - (float)yaw_rounds * 360.0f;

                GimbalLG::set_target_angle(gimbal_pitch_target_angle_, yaw_target_angle_cropped);
            } else if(Remote::rc.s1 == Remote::S_DOWN){
                /// TODO: Add vision mode
                GimbalLG::set_mode(GimbalLG::FORCED_RELAX_MODE);
            }
        } else {
            GimbalLG::set_mode(GimbalLG::FORCED_RELAX_MODE);
        }

        /*** ----------------------------------  Shoot  --------------------------------- ***/

        if (!InspectorS::gimbal_failure() && !InspectorS::remote_failure()) {
            if (Remote::rc.s1 == Remote::S_UP) {
                /// TODO: Safe mode
            } else if (Remote::rc.s1 == Remote::S_MIDDLE) {
                /// TODO: MANUAL MODE
            } else if (Remote::rc.s1 == Remote::S_DOWN) {
                /// TODO: VISION MODE
            }
        }

        /*** ---------------------------------- Chassis --------------------------------- ***/

        if (!InspectorS::chassis_failure() && !InspectorS::remote_failure()) {
            if (Remote::rc.s1 == Remote::S_UP) {
                SChassisLG::set_mode(SChassisLG::FORCED_RELAX_MODE);
            } else if (Remote::rc.s1 == Remote::S_MIDDLE) {
                SChassisLG::set_mode(SChassisLG::MANUAL_MODE);
                SChassisLG::set_velocity(Remote::rc.ch2 * UserS::chassis_v);
            } else {
                SChassisLG::set_mode(SChassisLG::AUTO_MODE);
            }
        } else {
            SChassisLG::set_mode(SChassisLG::FORCED_RELAX_MODE);
        }

        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}

//void UserS::set_mode(UserS::sentry_mode_t mode) {
//    if (mode == sentryMode) return;
//
//    sentryMode = mode;
//
//    if (sentryMode == FORCED_RELAX_MODE) GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
//    else {
//        GimbalLG::set_action(GimbalLG::SENTRY_MODE);
//        if (sentryMode == AUTO_MODE) {
//            // Resume the thread
//            blind_mode_start_time = SYSTIME;
//            chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
//            if (!vitualUserThread.started) {
//                vitualUserThread.started = true;
//                chSchWakeupS(vitualUserThreadReference.getInner(), 0);
//            }
//            chSysUnlock();  /// --- EXIT S-Locked state ---
//        }
//    }
//}

//void UserS::VitualUserThread::main() {
//
//    setName("Sentry_Gimbal_Auto");
//
//    while (!shouldTerminate()) {
//
//        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
//        if (sentryMode != AUTO_MODE) {
//            started = false;
//            chSchGoSleepS(CH_STATE_SUSPENDED);
//        }
//        chSysUnlock();  /// --- EXIT S-Locked state ---
//
//        Vision::send_gimbal(GimbalLG::get_accumulated_angle(GimbalLG::YAW),
//                            GimbalLG::get_accumulated_angle(GimbalLG::PITCH));
//
////        VisionPort::send_enemy_color(Referee::game_robot_state.robot_id < 10);
//
//        enemy_spotted = WITHIN_RECENT_TIME(Vision::last_update_time, 50);
//
//        if (enemy_spotted){
//            LOG("%.2f, %.2f   yaw: %.2f, pitch: %%.2f, distance: %.2f", GimbalLG::get_accumulated_angle(GimbalLG::YAW), GimbalLG::get_accumulated_angle(GimbalLG::PITCH), Vision::enemy_info.yaw_delta, Vision::enemy_info.pitch_delta, Vision::enemy_info.dist);
//        }
//
//        /*** ---------------------------------- Gimbal + Shooter --------------------------------- ***/
//
//        if (v_user_mode == FINAL_AUTO_MODE && enemy_spotted) {
//            float yaw_delta = Vision::enemy_info.yaw_delta + 17 - gimbal_yaw_target_angle_,
//                    pitch_delta = Vision::enemy_info.pitch_delta - 12 - gimbal_pitch_target_angle_;
//
//            if (!ABS_IN_RANGE(yaw_delta, GIMBAL_YAW_TARGET_FAST_TRIGGER)) {
//                gimbal_yaw_target_angle_ +=
//                        SIGN(yaw_delta) * (yaw_sensitivity[TARGET_FAST] * AUTO_CONTROL_INTERVAL / 1000.0f);
//            } else {
//                gimbal_yaw_target_angle_ +=
//                        SIGN(yaw_delta) * (yaw_sensitivity[TARGET_SLOW] * AUTO_CONTROL_INTERVAL / 1000.0f);
//            }
//
//            fire = (ABS_IN_RANGE(yaw_delta, GIMBAL_YAW_SHOOT_TRIGGER_ANGLE) && ABS_IN_RANGE(pitch_delta, GIMBAL_PIT_SHOOT_TRIGGER_ANGLE));
//
//
//            if (!ABS_IN_RANGE(pitch_delta, GIMBAL_PITCH_TARGET_FAST_TRIGGER))
//                gimbal_pitch_target_angle_ +=
//                        SIGN(pitch_delta) * (pitch_sensitivity[TARGET_FAST] * AUTO_CONTROL_INTERVAL / 1000.0f);
//            else
//                gimbal_pitch_target_angle_ +=
//                        SIGN(pitch_delta) * (pitch_sensitivity[TARGET_SLOW] * AUTO_CONTROL_INTERVAL / 1000.0f);
//
//            VAL_CROP(gimbal_yaw_target_angle_, gimbal_yaw_max_angle, gimbal_yaw_min_angle);
//            VAL_CROP(gimbal_pitch_target_angle_, gimbal_pitch_max_angle, gimbal_pitch_min_angle);
//
//        } else if ((v_user_mode == FINAL_AUTO_MODE && !enemy_spotted) || v_user_mode == CRUISING_ONLY_MODE) {
//
//            if (gimbal_yaw_target_angle_ < yaw_terminal){
//                gimbal_yaw_target_angle_ +=
//                        (yaw_sensitivity[CRUISING] * AUTO_CONTROL_INTERVAL / 1000.0f);
//            }else{
//                gimbal_yaw_target_angle_ -=
//                        (yaw_sensitivity[CRUISING] * AUTO_CONTROL_INTERVAL / 1000.0f);
//            }
//
//            if (gimbal_pitch_target_angle_ < pitch_terminal){
//                gimbal_pitch_target_angle_ +=
//                        (pitch_sensitivity[CRUISING] * AUTO_CONTROL_INTERVAL / 1000.0f);
//            }else{
//                gimbal_pitch_target_angle_ -=
//                        (pitch_sensitivity[CRUISING] * AUTO_CONTROL_INTERVAL / 1000.0f);
//            }
//
//            if (gimbal_yaw_target_angle_ >= gimbal_yaw_max_angle) yaw_terminal = gimbal_yaw_min_angle;
//            else if (gimbal_yaw_target_angle_ <= gimbal_yaw_min_angle) yaw_terminal = gimbal_yaw_max_angle;
//
//            if (gimbal_pitch_target_angle_ >= gimbal_pitch_max_angle) pitch_terminal = gimbal_pitch_min_angle;
//            else if (gimbal_pitch_target_angle_ <= gimbal_pitch_min_angle) pitch_terminal = gimbal_pitch_max_angle;
//
//            fire = false;
//        }
//
//        GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pitch_target_angle_, 0);
//
//        /*** ---------------------------------- Chassis --------------------------------- ***/
//
//        if ((v_user_mode == FINAL_AUTO_MODE) && (enemy_spotted || Referee::robot_hurt.armor_id > 0)) {
//            if (!SChassisLG::get_escaping_status()) {
//                SChassisLG::start_escaping();
//            }
//        }
//
//        sleep(TIME_MS2I(AUTO_CONTROL_INTERVAL));
//    }
//
//}
//
//void UserS::VitualUserThread::set_v_user_mode(UserS::VitualUserThread::vitual_user_mode_t mode) {
//    v_user_mode = mode;
//}
