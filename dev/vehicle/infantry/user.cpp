//
// Created by liuzikai on 2019-06-25.
//

#include "user.h"


/// Gimbal Config
float User::gimbal_rc_yaw_max_speed = 90;  // [degree/s]
float User::gimbal_pc_yaw_sensitivity[3] = {20000, 55000, 100000};  // [Ctrl, Normal, Shift] [degree/s]

float User::gimbal_pc_pitch_sensitivity[3] = {8000, 12000,
                                              12000};   // rotation speed when mouse moves fastest [degree/s]
float User::gimbal_pitch_min_angle = -10; // down range for pitch [degree]
float User::gimbal_pitch_max_angle = 45; //  up range for pitch [degree]

/// Chassis Config
float User::chassis_v_left_right = 1000.0f;  // [mm/s]
float User::chassis_v_forward = 2000.0f;     // [mm/s]
float User::chassis_v_backward = 2000.0f;    // [mm/s]

float User::chassis_pc_shift_ratio = 1.5f;  // 150% when Shift is pressed
float User::chassis_pc_ctrl_ratio = 0.5;    // 50% when Ctrl is pressed

Remote::key_t User::chassis_dodge_switch = Remote::KEY_X;
unsigned User::chassis_dodge_light_index = 3;

/// Shoot Config
float User::shoot_launch_left_count = 5;
float User::shoot_launch_right_count = 999;

float User::shoot_lanuch_speed = 5.0f;

float User::shoot_common_duty_cycle = 0.8;

unsigned User::shoot_remain_bullet_data_index = 1;

Remote::key_t User::shoot_fw_switch = Remote::KEY_Z;
unsigned User::shoot_fw_status_light_index = 0;

/// Other Config
unsigned User::high_speed_light_index = 5;
unsigned User::low_speed_light_index = 4;


/// Variables
User::UserThread User::userThread;
User::UserActionThread User::userActionThread;
User::BulletIncrementThread User::bulletIncrementThread;
User::ClientDataSendingThread User::clientDataSendingThread;

void User::start(tprio_t user_thd_prio, tprio_t user_action_thd_prio, tprio_t bullet_increment_thd_prio, tprio_t client_data_sending_thd_prio) {
    userThread.start(user_thd_prio);
    userActionThread.start(user_action_thd_prio);
    bulletIncrementThread.start(bullet_increment_thd_prio);
    clientDataSendingThread.start(client_data_sending_thd_prio);
}

void User::UserThread::main() {
    setName("User");
    while (!shouldTerminate()) {

        /*** ---------------------------------- Gimbal --------------------------------- ***/

        if (!Inspector::remote_failure() && !Inspector::chassis_failure() && !Inspector::gimbal_failure()) {

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

            } /*else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                /// Remote - Vision

                GimbalLG::set_action(GimbalLG::VISION_MODE);

            }*/ else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);

                float yaw_sensitivity, pitch_sensitivity;
                if (Remote::key.ctrl) {
                    yaw_sensitivity = gimbal_pc_yaw_sensitivity[0];
                    pitch_sensitivity = gimbal_pc_pitch_sensitivity[0];

                    Referee::set_client_light(high_speed_light_index, true);
                    Referee::set_client_light(low_speed_light_index, false);
                } else if (Remote::key.shift) {
                    yaw_sensitivity = gimbal_pc_yaw_sensitivity[2];
                    pitch_sensitivity = gimbal_pc_pitch_sensitivity[2];

                    Referee::set_client_light(high_speed_light_index, false);
                    Referee::set_client_light(low_speed_light_index, true);
                } else {
                    yaw_sensitivity = gimbal_pc_yaw_sensitivity[1];
                    pitch_sensitivity = gimbal_pc_pitch_sensitivity[1];

                    Referee::set_client_light(high_speed_light_index, false);
                    Referee::set_client_light(low_speed_light_index, false);
                }
                // Referee client data will be sent by ClientDataSendingThread

                gimbal_yaw_target_angle_ += -Remote::mouse.x * (yaw_sensitivity * USER_THREAD_INTERVAL / 1000.0f);
                // mouse.x use right as positive direction, while GimbalLG use CCW (left) as positive direction


                gimbal_pc_pitch_target_angle_ +=
                        -Remote::mouse.y * (pitch_sensitivity * USER_THREAD_INTERVAL / 1000.0f);
                // mouse.y use down as positive direction, while GimbalLG use CCW (left) as positive direction

                VAL_CROP(gimbal_pc_pitch_target_angle_, gimbal_pitch_max_angle, gimbal_pitch_min_angle);


                GimbalLG::set_target(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_);

            } else {
                /// Safe Mode
                GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
            }

        } else {  // Inspector::remote_failure() || Inspector::chassis_failure() || Inspector::gimbal_failure()
            /// Safe Mode
            GimbalLG::set_action(GimbalLG::FORCED_RELAX_MODE);
        }


        /*** ---------------------------------- Shoot --------------------------------- ***/

        if (!Inspector::remote_failure() && !Inspector::chassis_failure() && !Inspector::gimbal_failure()) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) /*||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN)*/) {

                /// Remote - Shoot with Scrolling Wheel

                if (Remote::rc.wheel > 0.5) {  // down
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(shoot_launch_left_count, 0);
                    }
                } else if (Remote::rc.wheel < -0.5) {  // up
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(shoot_launch_right_count, 0);
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

        if (!Inspector::remote_failure() && !Inspector::chassis_failure() && !Inspector::gimbal_failure()) {

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

        } else {  // Inspector::remote_failure() || Inspector::chassis_failure() || Inspector::gimbal_failure()
            /// Safe Mode
            ChassisLG::set_action(ChassisLG::FORCED_RELAX_MODE);
        }


        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}

void User::UserActionThread::main() {
    setName("User_Action");

    chEvtRegisterMask(&Remote::s_change_event, &s_change_listener, S_CHANGE_EVENTMASK);
    chEvtRegisterMask(&Remote::mouse_press_event, &mouse_press_listener, MOUSE_PRESS_EVENTMASK);
    chEvtRegisterMask(&Remote::mouse_release_event, &mouse_release_listener, MOUSE_RELEASE_EVENTMASK);
    chEvtRegisterMask(&Remote::key_press_event, &key_press_listener, KEY_PRESS_EVENTMASK);
    chEvtRegisterMask(&Remote::key_release_event, &key_release_listener, KEY_RELEASE_EVENTMASK);

    while (!shouldTerminate()) {

        eventmask_t events = chEvtWaitAny(MOUSE_PRESS_EVENTMASK | KEY_PRESS_EVENTMASK);

        /**
         * NOTICE: use flag instead of direct accessing variable in Remote, since event can be trigger by other key, etc
         */


        /// Mouse Press
        if (events & MOUSE_PRESS_EVENTMASK) {

            eventflags_t mouse_flag = chEvtGetAndClearFlags(&mouse_press_listener);

            /// Shoot
            if (ShootLG::get_friction_wheels_duty_cycle() == 0) {  // force start friction wheels
                ShootLG::set_friction_wheels(shoot_common_duty_cycle);
                Referee::set_client_light(shoot_fw_status_light_index, true);
            }
            if (mouse_flag & (1U << Remote::MOUSE_LEFT)) {
                ShootLG::shoot(shoot_launch_left_count, shoot_lanuch_speed);
            } else if (mouse_flag & (1U << Remote::MOUSE_RIGHT)) {
                ShootLG::shoot(shoot_launch_right_count, shoot_lanuch_speed);
            }
        } else {  // releasing one while pressing another won't result in stopping
            if (events & MOUSE_RELEASE_EVENTMASK) {
                ShootLG::stop();
            }
        }

        /// Key Press
        if (events & KEY_PRESS_EVENTMASK) {

            eventflags_t key_flag = chEvtGetAndClearFlags(&key_press_listener);

            /// Chassis
            if (key_flag & (1U << chassis_dodge_switch)) {
                if (ChassisLG::get_action() == ChassisLG::FOLLOW_MODE) {
                    ChassisLG::set_action(ChassisLG::DODGE_MODE);
                    Referee::set_client_light(chassis_dodge_light_index, true);
                } else if (ChassisLG::get_action() == ChassisLG::DODGE_MODE) {
                    ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                    Referee::set_client_light(chassis_dodge_light_index, false);
                }
            }

            /// Shoot
            if (key_flag & (1U << shoot_fw_switch)) {
                if (ShootLG::get_friction_wheels_duty_cycle() > 0) {
                    ShootLG::set_friction_wheels(0);
                    Referee::set_client_light(shoot_fw_status_light_index, false);
                } else {
                    ShootLG::set_friction_wheels(shoot_common_duty_cycle);
                    Referee::set_client_light(shoot_fw_status_light_index, true);
                }
            }
        }

        // If more event type is added, remember to modify chEvtWaitAny() above

        // Referee client data will be sent by ClientDataSendingThread

    }
}

void User::BulletIncrementThread::main() {
    setName("User_Bullet");

    chEvtRegisterMask(&Referee::data_received_event, &data_received_listener, DATA_RECEIVED_EVENTMASK);

    while (!shouldTerminate()) {
        chEvtWaitAny(DATA_RECEIVED_EVENTMASK);
        eventflags_t flags = chEvtGetAndClearFlags(&data_received_listener);
        if (flags == Referee::SUPPLY_PROJECTILE_ACTION_CMD_ID &&
            Referee::supply_projectile_action.supply_robot_id == Referee::get_self_id() &&
            Referee::supply_projectile_action.supply_projectile_step == 2  // bullet fall
                ) {
            ShootLG::increment_bullet((int) (Referee::supply_projectile_action.supply_projectile_num * 1.0f));
        }
    }
}

void User::ClientDataSendingThread::main() {
    setName("User_Client");
    while (!shouldTerminate()) {

        Referee::set_client_number(shoot_remain_bullet_data_index, ShootLG::get_bullet_count());

        Referee::send_data(Referee::CLIENT);
        sleep(TIME_MS2I(CLIENT_DATA_SENDING_THREAD_INTERVAL));
    }
}