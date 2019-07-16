//
// Created by liuzikai on 2019-06-25.
//

#include "sentry_user.h"


/// Gimbal Config
float User::yaw_sensitivity[3] = {40, 60, 90};  // [Slow, Normal, Fast] [degree/s]
float User::pitch_sensitivity[3] = {20, 30, 40};   // [Slow, Normal, Fast] [degree/s]
float User::gimbal_yaw_min_angle = -160; // left range for yaw [degree]
float User::gimbal_yaw_max_angle = 160; // right range for yaw [degree]
float User::gimbal_pitch_min_angle = -10; // down range for pitch [degree]
float User::gimbal_pitch_max_angle = 45; //  up range for pitch [degree]

/// Chassis Config
float User::chassis_v = 1000.0f;  // [mm/s]

/// Shoot Config
float User::shoot_launch_left_count = 5;
float User::shoot_launch_right_count = 999;

float User::shoot_launch_speed = 5.0f;

float User::shoot_common_duty_cycle = 0.6;

Remote::key_t User::shoot_fw_switch = Remote::KEY_Z;


/// Variables
User::UserThread User::userThread;

void User::start(tprio_t user_thd_prio) {
    userThread.start(user_thd_prio);
}

void User::UserThread::main() {
    setName("User");
    while (!shouldTerminate()) {

        /*** ---------------------------------- Gimbal --------------------------------- ***/

        if (!Inspector::remote_failure() && !Inspector::gimbal_failure()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

                /// Remote - Yaw + Pitch

                GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);

                if (Remote::rc.ch0 > 0) gimbal_yaw_target_angle_ = -Remote::rc.ch0 * gimbal_yaw_max_angle;
                else gimbal_yaw_target_angle_ = Remote::rc.ch0 * gimbal_yaw_min_angle;  // GIMBAL_YAW_MIN_ANGLE is negative
                // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction

                if (Remote::rc.ch1 > 0) gimbal_pitch_target_angle_ = Remote::rc.ch1 * gimbal_pitch_max_angle;
                else gimbal_pitch_target_angle_ = -Remote::rc.ch1 * gimbal_pitch_min_angle;  // GIMBAL_PITCH_MIN_ANGLE is negative
                // ch1 use up as positive direction, while GimbalLG also use up as positive direction


                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pitch_target_angle_);

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {

                /// Remote - Yaw

                GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);


                gimbal_yaw_target_angle_ +=
                        -Remote::rc.ch0 * (yaw_sensitivity[CRUISING] * USER_THREAD_INTERVAL / 1000.0f);
                // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction

            }else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN){

                /// Remote - Vision

                GimbalLG::set_action(GimbalLG::VISION_MODE);

                GimbalLG::set_target(VisionPort::enemy_info.yaw_angle, VisionPort::enemy_info.pitch_angle);

            }else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode
                if (VisionPort::enemy_info.distance == 0){
                    
                    GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);
                    
                }
                VAL_CROP(gimbal_pitch_target_angle_, gimbal_pitch_max_angle, gimbal_pitch_min_angle);

                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pitch_target_angle_);

            } else {
                /// Safe Mode
                GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
            }

        } else {  // Inspector::remote_failure() || Inspector::chassis_failure() || Inspector::gimbal_failure()
            /// Safe Mode
            GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
        }


        /*** ---------------------------------- Shoot --------------------------------- ***/

        if (!Inspector::remote_failure() /*&& !Inspector::chassis_failure()*/ && !Inspector::gimbal_failure()) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) /*||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN)*/) {

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

                // UserActionThread handles launching and status of friction wheels

            } else {
                /// Safe Mode
                ShootLG::stop();
                ShootLG::set_friction_wheels(0);
            }

        } else {  // Inspector::remote_failure() || Inspector::chassis_failure() || Inspector::gimbal_failure()
            /// Safe Mode
            ShootLG::stop();
            ShootLG::set_friction_wheels(0);
        }

        /*** ---------------------------------- Chassis --------------------------------- ***/

        if (!Inspector::remote_failure() && !Inspector::chassis_failure()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE) {

                /// Remote - MANUAL_MODE
                SChassisLG::set_mode(SChassisLG::MANUAL_MODE);
                SChassisLG::set_manual_dest(Remote::rc.ch2 * chassis_v + SChassisLG::get_manual_dest());

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// AUTO - FINAL_MODE

                SChassisLG::set_mode(SChassisLG::FINAL_AUTO_MODE);

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                /// Remote - Chassis Stop

                SChassisLG::set_mode(SChassisLG::FORCED_RELAX_MODE);

            }


            /// Final
            sleep(TIME_MS2I(USER_THREAD_INTERVAL));
        }
    }
}