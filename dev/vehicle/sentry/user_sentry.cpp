//
// Created by Kerui Zhu on 2019-7-16
//

#include "user_sentry.h"

UserS::sentry_mode_t UserS::sentryMode = FORCED_RELAX_MODE;

/// Gimbal Config
float UserS::yaw_sensitivity[3] = {40, 60, 90};  // [Slow, Normal, Fast] [degree/s]
float UserS::pitch_sensitivity[3] = {20, 30, 40};   // [Slow, Normal, Fast] [degree/s]
float UserS::gimbal_yaw_target_angle_ = 0;
float UserS::gimbal_pitch_target_angle_ = 0;
float UserS::gimbal_yaw_min_angle = -160; // left range for yaw [degree]
float UserS::gimbal_yaw_max_angle = 160; // right range for yaw [degree]
float UserS::gimbal_pitch_min_angle = -50; // down range for pitch [degree]
float UserS::gimbal_pitch_max_angle = 10; //  up range for pitch [degree]

/// Chassis Config
float UserS::chassis_v = 1000.0f;  // [mm/s]

/// Shoot Config
float UserS::shoot_launch_left_count = 5;
float UserS::shoot_launch_right_count = 999;

float UserS::shoot_launch_speed = 5.0f;

float UserS::shoot_common_duty_cycle = 0.6;

bool UserS::fire = false;


/// Variables
UserS::UserThread UserS::userThread;
UserS::VitualUserThread UserS::vitualUserThread;
chibios_rt::ThreadReference UserS::vitualUserThreadReference;

void UserS::start(tprio_t user_thd_prio, tprio_t v_user_thd_prio) {
    userThread.start(user_thd_prio);
    vitualUserThread.started = true;
    vitualUserThreadReference = vitualUserThread.start(v_user_thd_prio);
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

        /*** ---------------------------------- Gimbal --------------------------------- ***/

        if (!InspectorS::remote_failure() && !InspectorS::gimbal_failure()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

                /// Remote - Yaw + Pitch

                set_mode(REMOTE_MODE);

                if (Remote::rc.ch0 > 0) gimbal_yaw_target_angle_ = -Remote::rc.ch0 * gimbal_yaw_max_angle;
                else
                    gimbal_yaw_target_angle_ =
                            Remote::rc.ch0 * gimbal_yaw_min_angle;  // GIMBAL_YAW_MIN_ANGLE is negative
                // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction

                if (Remote::rc.ch1 > 0) gimbal_pitch_target_angle_ = Remote::rc.ch1 * gimbal_pitch_max_angle;
                else
                    gimbal_pitch_target_angle_ =
                            -Remote::rc.ch1 * gimbal_pitch_min_angle;  // GIMBAL_PITCH_MIN_ANGLE is negative
                // ch1 use up as positive direction, while GimbalLG also use up as positive direction


                SGimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pitch_target_angle_);

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {

                /// Remote - Yaw

                set_mode(REMOTE_MODE);


                gimbal_yaw_target_angle_ +=
                        -Remote::rc.ch0 * (yaw_sensitivity[CRUISING] * USER_THREAD_INTERVAL / 1000.0f);
                // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction
                gimbal_pitch_target_angle_ +=
                        Remote::rc.ch1 * (pitch_sensitivity[CRUISING] * USER_THREAD_INTERVAL / 1000.0f);
                // ch1 use up as positive direction, while GimbalLG use up as positive direction

                VAL_CROP(gimbal_yaw_target_angle_, gimbal_yaw_max_angle, gimbal_yaw_min_angle);
                VAL_CROP(gimbal_pitch_target_angle_, gimbal_pitch_max_angle, gimbal_pitch_min_angle);

                SGimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pitch_target_angle_);

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                set_mode(AUTO_MODE);
                vitualUserThread.set_v_user_mode(VitualUserThread::CRUISING_ONLY_MODE);

            } else if (Remote::rc.s1 == Remote::S_DOWN && Remote::rc.s2 == Remote::S_UP) {

                set_mode(AUTO_MODE);
                vitualUserThread.set_v_user_mode(VitualUserThread::VISION_ONLY_MODE);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                set_mode(AUTO_MODE);
                vitualUserThread.set_v_user_mode(VitualUserThread::FINAL_AUTO_MODE);

            } else {
                /// Safe Mode
                set_mode(FORCED_RELAX_MODE);
            }

        } else {  // InspectorS::remote_failure() || InspectorS::chassis_failure() || InspectorS::gimbal_failure()
            /// Safe Mode
            set_mode(FORCED_RELAX_MODE);
        }


        /*** ---------------------------------- Shoot --------------------------------- ***/

        if (!InspectorS::remote_failure() /*&& !InspectorS::chassis_failure()*/ && !InspectorS::gimbal_failure()) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {

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

                ShootLG::set_friction_wheels(shoot_common_duty_cycle);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                if (fire) {
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(shoot_launch_right_count, shoot_launch_speed);
                    }
                } else {
                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
                        ShootLG::stop();
                    }
                }

            } else {
                /// Safe Mode
                ShootLG::stop();
                ShootLG::set_friction_wheels(0);
            }

        } else {  // InspectorS::remote_failure() || InspectorS::chassis_failure() || InspectorS::gimbal_failure()
            /// Safe Mode
            ShootLG::stop();
            ShootLG::set_friction_wheels(0);
        }

        /*** ---------------------------------- Chassis --------------------------------- ***/

        if (!InspectorS::remote_failure() && !InspectorS::chassis_failure()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE ||
                (Remote::rc.s1 == Remote::S_DOWN && Remote::rc.s2 == Remote::S_UP)) {

                /// Remote - MANUAL_MODE
                SChassisLG::set_mode(SChassisLG::MANUAL_MODE);
                SChassisLG::set_manual_dest(
                        Remote::rc.ch2 * chassis_v * USER_THREAD_INTERVAL / 1000.0f + SChassisLG::get_manual_dest());

            } else if (Remote::rc.s1 == Remote::S_DOWN && Remote::rc.s2 == Remote::S_MIDDLE) {

                /// Remote - SHUTTLE_MODE
                SChassisLG::set_mode(SChassisLG::SHUTTLE_MODE);

            } else if (Remote::rc.s1 == Remote::S_DOWN && Remote::rc.s2 == Remote::S_DOWN) {

                /// AUTO - FINAL_MODE

                SChassisLG::set_mode(SChassisLG::FINAL_AUTO_MODE);

            } else {

                /// Remote - Chassis Stop

                SChassisLG::set_mode(SChassisLG::FORCED_RELAX_MODE);

            }

        }

        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}

void UserS::set_mode(UserS::sentry_mode_t mode) {
    if (mode == sentryMode) return;

    sentryMode = mode;

    if (sentryMode == FORCED_RELAX_MODE) SGimbalLG::set_action(SGimbalLG::FORCED_RELAX_MODE);
    else {
        SGimbalLG::set_action(SGimbalLG::ABS_ANGLE_MODE);
        if (sentryMode == AUTO_MODE) {
            // Resume the thread
            chSysLock();  /// ---------------------------------- Enter Critical Zone ----------------------------------
            if (!vitualUserThread.started) {
                vitualUserThread.started = true;
                chSchWakeupS(vitualUserThreadReference.getInner(), 0);
            }
            chSysUnlock();  /// ---------------------------------- Exit Critical Zone ----------------------------------
        }
    }
}

void UserS::VitualUserThread::main() {

    setName("Sentry_Gimbal_Auto");

    while (!shouldTerminate()) {
        chSysLock();  /// ---------------------------------- Enter Critical Zone ----------------------------------
        if (sentryMode != AUTO_MODE) {
            started = false;
            chSchGoSleepS(CH_STATE_SUSPENDED);
        }
        chSysUnlock();  /// ---------------------------------- Exit Critical Zone ----------------------------------

        VisionPort::send_gimbal(SGimbalLG::get_accumulated_angle(SGimbalLG::YAW),
                                SGimbalLG::get_accumulated_angle(SGimbalLG::PITCH));

        VisionPort::send_enemy_color(Referee::game_robot_state.robot_id < 10);

        enemy_spotted = SYSTIME - VisionPort::last_update_time > 50;

        /*** ---------------------------------- Gimbal + Shooter --------------------------------- ***/

        if ((v_user_mode == FINAL_AUTO_MODE && enemy_spotted) || v_user_mode == VISION_ONLY_MODE) {

            float yaw_delta = VisionPort::enemy_info.yaw_angle - gimbal_yaw_target_angle_,
                    pitch_delta = VisionPort::enemy_info.pitch_angle - gimbal_pitch_target_angle_;

            if (!ABS_IN_RANGE(yaw_delta, GIMBAL_YAW_TARGET_FAST_TRIGGER)) {
                gimbal_yaw_target_angle_ +=
                        -SIGN(yaw_delta) * (yaw_sensitivity[TARGET_FAST] * AUTO_CONTROL_INTERVAL / 1000.0f);
                fire = false;
            } else {
                gimbal_yaw_target_angle_ +=
                        -SIGN(yaw_delta) * (yaw_sensitivity[TARGET_SLOW] * AUTO_CONTROL_INTERVAL / 1000.0f);
                fire = true;
            }


            if (!ABS_IN_RANGE(pitch_delta, GIMBAL_PITCH_TARGET_FAST_TRIGGER))
                gimbal_pitch_target_angle_ +=
                        SIGN(pitch_delta) * (pitch_sensitivity[TARGET_FAST] * AUTO_CONTROL_INTERVAL / 1000.0f);
            else
                gimbal_pitch_target_angle_ +=
                        SIGN(pitch_delta) * (pitch_sensitivity[TARGET_SLOW] * AUTO_CONTROL_INTERVAL / 1000.0f);

            VAL_CROP(gimbal_yaw_target_angle_, gimbal_yaw_max_angle, gimbal_yaw_min_angle);
            VAL_CROP(gimbal_pitch_target_angle_, gimbal_pitch_max_angle, gimbal_pitch_min_angle);

        } else if ((v_user_mode == FINAL_AUTO_MODE && !enemy_spotted) || v_user_mode == CRUISING_ONLY_MODE) {
            if (gimbal_yaw_target_angle_ < yaw_terminal)
                gimbal_yaw_target_angle_ +=
                        -(yaw_sensitivity[CRUISING] * AUTO_CONTROL_INTERVAL / 1000.0f);
            else
                gimbal_yaw_target_angle_ -=
                        -(yaw_sensitivity[CRUISING] * AUTO_CONTROL_INTERVAL / 1000.0f);

            if (gimbal_pitch_target_angle_ < pitch_terminal)
                gimbal_pitch_target_angle_ +=
                        (pitch_sensitivity[CRUISING] * AUTO_CONTROL_INTERVAL / 1000.0f);
            else
                gimbal_pitch_target_angle_ -=
                        (pitch_sensitivity[CRUISING] * AUTO_CONTROL_INTERVAL / 1000.0f);

            if (gimbal_yaw_target_angle_ >= gimbal_yaw_max_angle) yaw_terminal = gimbal_yaw_min_angle;
            else if (gimbal_yaw_target_angle_ <= gimbal_yaw_min_angle) yaw_terminal = gimbal_yaw_max_angle;

            if (gimbal_pitch_target_angle_ >= gimbal_pitch_max_angle) pitch_terminal = gimbal_pitch_min_angle;
            else if (gimbal_pitch_target_angle_ <= gimbal_pitch_min_angle) pitch_terminal = gimbal_pitch_max_angle;
        }

        SGimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pitch_target_angle_);

        /*** ---------------------------------- Chassis --------------------------------- ***/

        if ((v_user_mode == FINAL_AUTO_MODE) && (enemy_spotted || Referee::robot_hurt.armor_id > 0)) {
            if (!SChassisLG::get_escaping_status()) {
                SChassisLG::start_escaping();
            }
        }


        sleep(TIME_MS2I(AUTO_CONTROL_INTERVAL));
    }

}

void UserS::VitualUserThread::set_v_user_mode(UserS::VitualUserThread::vitual_user_mode_t mode) {
    if (v_user_mode == mode)
        return;
    v_user_mode = mode;
}
