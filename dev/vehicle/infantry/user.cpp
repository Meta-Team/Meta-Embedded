//
// Created by liuzikai on 2019-06-25.
//

#include "user.h"

User::UserThread User::userThread;

void User::start(tprio_t prio) {
    userThread.start(prio);
}

void User::UserThread::main() {
    setName("User");
    while (!shouldTerminate()) {

        /*** ---------------------------------- Gimbal --------------------------------- ***/

        if (!Inspector::remote_failure() && !Inspector::gimbal_failure()) {

            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {

                /// Remote - Yaw + Pitch

                GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);


                gimbal_rc_yaw_target_angle += -Remote::rc.ch0 * (GIMBAL_RC_YAW_MAX_SPEED * USER_THREAD_INTERVAL / 1000.0f);
                // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction

                float pitch_target;
                if (Remote::rc.ch1 > 0) pitch_target = Remote::rc.ch1 * GIMBAL_PITCH_MAX_ANGLE;
                else pitch_target = -Remote::rc.ch1 * GIMBAL_PITCH_MIN_ANGLE;  // GIMBAL_PITCH_MIN_ANGLE is negative
                // ch1 use up as positive direction, while GimbalLG also use up as positive direction


                GimbalLG::set_target(gimbal_rc_yaw_target_angle, pitch_target);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);


                gimbal_pc_yaw_target_angle +=
                                   -Remote::mouse.x * (GIMBAL_PC_YAW_SENSITIVITY * USER_THREAD_INTERVAL / 1000.0f);
                // mouse.x use right as positive direction, while GimbalLG use CCW (left) as positive direction


                gimbal_pc_pitch_target_angle +=
                        -Remote::mouse.y * (GIMBAL_PC_PITCH_SENSITIVITY * USER_THREAD_INTERVAL / 1000.0f);
                // mouse.y use down as positive direction, while GimbalLG use CCW (left) as positive direction

                if (gimbal_pc_pitch_target_angle < GIMBAL_PITCH_MIN_ANGLE)
                    gimbal_pc_pitch_target_angle = GIMBAL_PITCH_MIN_ANGLE;
                if (gimbal_pc_pitch_target_angle > GIMBAL_PITCH_MAX_ANGLE)
                    gimbal_pc_pitch_target_angle = GIMBAL_PITCH_MAX_ANGLE;


                GimbalLG::set_target(gimbal_pc_yaw_target_angle, gimbal_pc_pitch_target_angle);

            } else {
                /// Safe Mode
                GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
            }

        } else {  // Inspector::remote_failure() || Inspector::gimbal_failure()
            /// Safe Mode
            GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
        }


        /*** ---------------------------------- Shoot --------------------------------- ***/

        if (!Inspector::remote_failure() && !Inspector::gimbal_failure()) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {

                /// Remote - Shoot with Scrolling Wheel

                if (Remote::rc.wheel < -0.5) {  // down
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(SHOOT_LAUNCH_LEFT_COUNT);
                    }
                } else if (Remote::rc.wheel > 0.5) {  // up
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(SHOOT_LAUNCH_RIGHT_COUNT);
                    }
                } else {
                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
                        ShootLG::stop();
                    }
                }

                ShootLG::set_friction_wheels(SHOOT_COMMON_DUTY_CYCLE);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                if (pc_mouse_left_pressed != Remote::mouse.press_left ||
                    pc_mouse_right_pressed != Remote::mouse.press_right) {  // mouse buttons are pressed or released

                    if (Remote::mouse.press_left) {
                        ShootLG::shoot(SHOOT_LAUNCH_LEFT_COUNT);
                    } else if (Remote::mouse.press_right) {
                        ShootLG::shoot(SHOOT_LAUNCH_RIGHT_COUNT);
                    } else {
                        ShootLG::stop();
                    }

                    pc_mouse_left_pressed = Remote::mouse.press_left;
                    pc_mouse_right_pressed = Remote::mouse.press_right;
                }

                if (pc_z_pressed != Remote::key.z) {  // key z is pressed or released
                    // TODO: echo status to user
                    if (ShootLG::get_friction_wheels_duty_cycle() > 0) {
                        ShootLG::set_friction_wheels(0);
                    } else {
                        ShootLG::set_friction_wheels(SHOOT_COMMON_DUTY_CYCLE);
                    }

                    pc_z_pressed = Remote::key.z;
                }

            } else {
                /// Safe Mode
                ShootLG::stop();
                ShootLG::set_friction_wheels(0);
            }

        } else {  // Inspector::remote_failure() || Inspector::gimbal_failure()
            /// Safe Mode
            ShootLG::stop();
            ShootLG::set_friction_wheels(0);
        }

        /*** ---------------------------------- Chassis --------------------------------- ***/

        if (!Inspector::remote_failure() && !Inspector::chassis_failure()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

                /// Remote - Chassis Move + Chassis Follow
                ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                ChassisLG::set_target(Remote::rc.ch2 * CHASSIS_COMMON_VX,  // Both use right as positive direction
                                      Remote::rc.ch3 * CHASSIS_COMMON_VX   // Both use up    as positive direction
                );

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {

                /// Remote - Chassis Move + Chassis Dodge
                ChassisLG::set_action(ChassisLG::DODGE_MODE);
                ChassisLG::set_target(Remote::rc.ch2 * CHASSIS_COMMON_VX,  // Both use right as positive direction
                                      Remote::rc.ch3 * CHASSIS_COMMON_VX   // Both use up    as positive direction
                );
            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                if (ChassisLG::get_action() == ChassisLG::FORCED_RELAX_MODE) {
                    // Enter PC Mode from other mode, re-enable ChassisLG
                    ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                }

                if (pc_x_pressed != Remote::key.x) {  // key x is pressed or released
                    // TODO: echo status to user
                    if (Remote::key.x) {  // enter dodge mode
                        ChassisLG::set_action(ChassisLG::DODGE_MODE);
                    } else {  // exit dodge mode
                        ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                    }
                    pc_x_pressed = Remote::key.x;
                }

                float target_vx, target_vy;

                if (Remote::key.w) target_vy = CHASSIS_COMMON_VY;
                else if (Remote::key.s) target_vy = -CHASSIS_COMMON_VY;
                else target_vy = 0;

                if (Remote::key.d) target_vx = CHASSIS_COMMON_VY;
                else if (Remote::key.a) target_vx = -CHASSIS_COMMON_VY;
                else target_vx = 0;

                if (Remote::key.ctrl) {
                    target_vx *= CHASSIS_PC_CTRL_RATIO;
                    target_vy *= CHASSIS_PC_CTRL_RATIO;
                }

                ChassisLG::set_target(target_vx, target_vy);

            } else {
                /// Safe Mode
                ChassisLG::set_action(ChassisLG::FORCED_RELAX_MODE);
            }

        } else {  // Inspector::remote_failure() || Inspector::chassis_failure()
            /// Safe Mode
            ChassisLG::set_action(ChassisLG::FORCED_RELAX_MODE);
        }


        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}