//
// Created by liuzikai on 2019-06-25.
//

#include "user_infantry.h"

#include "hardware_conf.h"

// FIXME: change as input parameter from main()

/// Vehicle Specific Configurations
#if defined(INFANTRY_THREE)                                                 /** Infantry #3 **/
#include "vehicle_infantry_three.h"
#elif defined(INFANTRY_FOUR)                                                /** Infantry #4 **/
#include "vehicle_infantry_four.h"
#elif defined(INFANTRY_FIVE)                                                /** Infantry #5 **/
#include "vehicle_infantry_five.h"
#else
#error "File main_infantry.cpp should only be used for Infantry #3, #4, #5."
#endif

#if ENABLE_REFEREE == TRUE
#include "referee_UI_logic.h"
#endif

//for debug use, otherwise please set it to 1.0f
#define SpeedAdjustCoefficient 0.7f
/// Gimbal Config
float UserI::gimbal_rc_yaw_max_speed = 180;  // [degree/s]
float UserI::gimbal_pc_yaw_sensitivity[3] = {100000, 200000, 300000};  // [Slow, Normal, Fast] [degree/s]

float UserI::gimbal_pc_pitch_sensitivity[3] = {20000, 50000, 60000};   // [Slow, Normal, Fast] [degree/s]

/// Chassis Config
float UserI::base_power = 40.0f;
float UserI::base_v_forward = 1500.0f;
float UserI::chassis_v_left_right = 1500.0f;   // [mm/s]
float UserI::chassis_v_forward = 1500.0f;     // [mm/s]
float UserI::chassis_v_backward = 1500.0f;    // [mm/s]

float UserI::chassis_pc_shift_ratio = 1.8f;  // 180% when Shift is pressed
float UserI::chassis_pc_ctrl_ratio = 0.5f;    // 50% when Ctrl is pressed

float UserI::shoot_feed_rate = 5.0f;   // [bullet/s]

// adjust for infantry4 in rmul2023 shanghai
float UserI::shoot_fw_speed[3] = {23300, 23300, 23300};  // [Slow, Normal, Fast] [deg/s]



/// Variables
float UserI::gimbal_yaw_target_angle_ = 0;
float UserI::gimbal_pc_pitch_target_angle_ = 0;
bool UserI::ignore_shoot_constraints = false;
UserI::UserThread UserI::userThread;
UserI::UserActionThread UserI::userActionThread;

void UserI::start(tprio_t user_thd_prio, tprio_t user_action_thd_prio) {
    userThread.start(user_thd_prio);
    userActionThread.start(user_action_thd_prio);
}

void UserI::UserThread::main() {
    setName("UserI");
    float pitch_target = 0;
    while (!shouldTerminate()) {
        //LOG("Heat=%d",Referee::power_heat.shooter_id1_17mm_cooling_heat);
        //LOG("Failure:Remote=%d,Chassis=%d,Gimbal=%d,gimbal_yaw_target_angle_=%.f,gimbal_yaw_feedback=%.f",InspectorI::remote_failure() ,InspectorI::chassis_failure() ,InspectorI::gimbal_failure(),gimbal_yaw_target_angle_,GimbalLG::get_feedback_angle(GimbalSKD::YAW));
        bool remote_not_failure = !InspectorI::remote_failure();
        bool chassis_not_failure = !InspectorI::chassis_failure();
        bool gimbal_not_failure = !InspectorI::gimbal_failure();
        /*** ---------------------------------- Gimbal --------------------------------- ***/
        if (remote_not_failure) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {

                /// Remote - Yaw + Pitch
#if ENABLE_AHRS
                GimbalLG::set_mode(GimbalLG::GIMBAL_REF_MODE);
#else
                GimbalLG::set_mode(GimbalLG::CHASSIS_REF_MODE);
#endif

                gimbal_yaw_target_angle_ +=
                        -Remote::rc.ch0 * (gimbal_rc_yaw_max_speed * USER_THREAD_INTERVAL / 1000.0f);
                // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction

                if (Remote::rc.ch1 > 0) pitch_target += (float) (Remote::rc.ch1 * GIMBAL_PITCH_MAX_ANGLE * 0.1);
                else
                    pitch_target -=
                            (float) (Remote::rc.ch1 * GIMBAL_PITCH_MIN_ANGLE * 0.1);  // GIMBAL_PITCH_MIN_ANGLE is negative
                // ch1 use up as positive direction, while GimbalLG also use up as positive direction

                VAL_CROP(pitch_target, GIMBAL_PITCH_MAX_ANGLE, GIMBAL_PITCH_MIN_ANGLE);
                GimbalLG::set_target_angle(gimbal_yaw_target_angle_, pitch_target);
#if ENABLE_VISION == TRUE
            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                /// Vision - Change bullet speed with right vertical handle

                /// Vision - Yaw + Pitch
                GimbalLG::set_mode(GimbalLG::VISION_MODE);

                // sync target with current feedback angle
                // because when in vision mode, it read from
                // otherwise when switching to normal mode(Remote - Yaw + Pitch), it will
                // rotate to the original YAW(gimbal_yaw_target_angle_)

                gimbal_yaw_target_angle_ = GimbalLG::get_feedback_angle(GimbalSKD::YAW);
                gimbal_pc_pitch_target_angle_ = GimbalLG::get_feedback_angle(GimbalSKD::PITCH);


                if (Remote::rc.ch1 > 0.5) VisionSKD::set_bullet_speed(VisionSKD::get_bullet_speed() - 0.001f);
                else if (Remote::rc.ch1 <= -0.5) VisionSKD::set_bullet_speed(VisionSKD::get_bullet_speed() + 0.001f);
#endif
            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode
#if ENABLE_VISION
                if (Remote::key.shift && Remote::key.v) {
                    VisionSKD::set_bullet_speed(VisionSKD::get_bullet_speed() + 0.001f);
                } else if (Remote::key.ctrl && Remote::key.v) {
                    VisionSKD::set_bullet_speed(VisionSKD::get_bullet_speed() - 0.001f);
                }
                if (Remote::mouse.press_right && VisionSKD::is_detected()) {

                    GimbalLG::set_mode(GimbalLG::VISION_MODE);

                    gimbal_yaw_target_angle_ = GimbalLG::get_feedback_angle(GimbalSKD::YAW);
                    gimbal_pc_pitch_target_angle_ = GimbalLG::get_feedback_angle(GimbalSKD::PITCH);

                } else {
#else
                if (!Remote::mouse.press_right) {
#endif
#if ENABLE_AHRS
                    GimbalLG::set_mode(GimbalLG::GIMBAL_REF_MODE);
#else
                    GimbalLG::set_mode(GimbalLG::CHASSIS_REF_MODE);
#endif

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

                    VAL_CROP(gimbal_pc_pitch_target_angle_, GIMBAL_PITCH_MAX_ANGLE, GIMBAL_PITCH_MIN_ANGLE);

                    GimbalLG::set_target_angle(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_);
                }

            } else {
                /// Safe Mode
                GimbalLG::set_mode(GimbalLG::FORCED_RELAX_MODE);
            }

        } else {  // InspectorI::remote_failure() || InspectorI::chassis_failure() || InspectorI::gimbal_failure()
            /// Safe Mode
            GimbalLG::set_mode(GimbalLG::FORCED_RELAX_MODE);
        }

        /*** ---------------------------------- Shoot --------------------------------- ***/
        if (remote_not_failure) {
            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE)) {

                /// Remote - Shoot with Scrolling Wheel

                ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);

                if (Remote::rc.wheel > 0.5) {  // down
                    ShootLG::set_shoot_speed(shoot_fw_speed[1]);
                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(ignore_shoot_constraints ? 999 : ShootLG::get_bullet_count_to_heat_limit(), shoot_feed_rate);
                    }
                } else if (Remote::rc.wheel < -0.5) {  // up
                    ShootLG::set_shoot_speed(shoot_fw_speed[1]);
                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {
                        ShootLG::shoot(999 /* unlimited */, shoot_feed_rate);
                    }
                } else {
                    if (ShootLG::get_shooter_state() != ShootLG::STOP) {
                        // stop the bullet loader
                        ShootLG::stop();
                    }
                }

                ShootLG::set_shoot_speed(shoot_fw_speed[1]);

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

                ShootLG::set_shoot_speed(shoot_fw_speed[1]);

            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode

                if (Remote::mouse.press_left) {
#if ENABLE_REFEREE == TRUE
                    // Read shoot limit
                    //TODO  check the version of referee system!!!, the current protocol might be deprecated
                    if (Referee::robot_state.shooter_id1_17mm_cooling_rate >= 40) {
                        shoot_feed_rate = Referee::robot_state.shooter_id1_17mm_cooling_rate / 10 * 1.25;
                    } else {
                        shoot_feed_rate = Referee::robot_state.shooter_id1_17mm_cooling_limit / 25;
                    }
#if ENABLE_VISION == TRUE
                    // removed in RMUC2023, since trajectory calculation is moved to Orin Nano, thus
                    // the calculation method in vision_scheduler.cpp, thus no shoot_update event will be broadcast
                    // and auto fire will not work
                    /*
                    if (!ignore_shoot_constraints && Remote::mouse.press_right) {
                        ShootLG::set_limit_mode(ShootLG::VISION_LIMITED_MODE);
                    } else {
                        ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
                    }
                     */
#endif
#else
                    ShootLG::set_limit_mode(ShootLG::UNLIMITED_MODE);
#endif
                    if (ShootLG::get_shoot_speed() == 0) {  // force start friction wheels
                        ShootLG::set_shoot_speed(shoot_fw_speed[1]);
                    }
                    ///
                    if (ShootLG::get_shooter_state() == ShootLG::STOP) {  // set target once
                        //ShootLG::shoot(ignore_shoot_constraints ? 999 : ShootLG::get_bullet_count_to_heat_limit(),
                        //               shoot_feed_rate);
                        ShootLG::shoot(999 ,shoot_feed_rate);
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

        /*** ---------------------------------- Chassis --------------------------------- ***/
        if (remote_not_failure) {

            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) ||
                (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN)) {
                /// Remote - Chassis Move + Chassis Follow
#if ENABLE_AHRS
                ChassisLG::set_mode(ChassisLG::GIMBAL_REF_MODE);
#else
                ChassisLG::set_mode(ChassisLG::CHASSIS_REF_MODE);
#endif
                ChassisLG::set_target(SpeedAdjustCoefficient*Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                                      (SpeedAdjustCoefficient*(Remote::rc.ch3 > 0 ?
                                       Remote::rc.ch3 * 1000 :
                                       Remote::rc.ch3 * 800))   // Both use up as positive direction
                );

            } /*else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {

                /// Remote - Chassis Move + Chassis Dodge
#if ENABLE_AHRS
                ChassisLG::set_mode(ChassisLG::DODGE);
#else
                ChassisLG::set_mode(ChassisLG::CHASSIS_REF_MODE);
#endif
                ChassisLG::set_target(Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                                      (Remote::rc.ch3 > 0 ?
                                       Remote::rc.ch3 * 1000 :
                                       Remote::rc.ch3 * 800)   // Both use up as positive direction
                );

            } */else if (Remote::rc.s1 == Remote::S_DOWN) {

                /// PC control mode: Chassis - Movement

                // Read current level information
#if ENABLE_REFEREE == TRUE
                chassis_v_forward = Referee::robot_state.chassis_power_limit * 0.9 / base_power * base_v_forward;
#else
                chassis_v_forward = 2000.0f;
#endif
                chassis_v_backward = chassis_v_forward;

                if (ChassisLG::get_mode() == ChassisLG::FORCE_RELAX_MODE) {
                    // Enter PC Mode from other mode, re-enable ChassisLG
#if ENABLE_AHRS
                    ChassisLG::set_mode(ChassisLG::GIMBAL_REF_MODE);
#else
                    ChassisLG::set_mode(ChassisLG::CHASSIS_REF_MODE);
#endif
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

                ChassisLG::set_target(SpeedAdjustCoefficient*target_vx, SpeedAdjustCoefficient*target_vy);

            } else {

                /// Safe Mode
                ChassisLG::set_mode(ChassisLG::FORCE_RELAX_MODE);
            }

        } else {  // InspectorI::remote_failure() || InspectorI::chassis_failure() || InspectorI::gimbal_failure()

            /// Safe Mode
            ChassisLG::set_mode(ChassisLG::FORCE_RELAX_MODE);
        }

#if ENABLE_REFEREE == TRUE
        /// Reset referee UI
        if (Remote::key.ctrl && Remote::key.shift && Remote::key.z) {
            RefereeUILG::reset();
        }
#endif

        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}

void UserI::UserActionThread::main() {
    setName("UserI_Action");

    chEvtRegisterMask(&Remote::key_press_event, &key_press_listener, EVENT_MASK(0));

    while (!shouldTerminate()) {
        chEvtWaitAny(ALL_EVENTS);

        /**
         * NOTICE: use flags instead of direct accessing variable in Remote, since event can be trigger by other key, etc
         */

        eventflags_t key_flag = chEvtGetAndClearFlags(&key_press_listener);

        if (key_flag & (1U << Remote::KEY_Z)) {
            if (Remote::key.ctrl && Remote::key.shift) {
#if ENABLE_REFEREE == TRUE
                // Ctrl + Shift + Z: reset referee UI
                RefereeUILG::reset();
#endif
            } else {
                // Z: chassis follow mode
#if ENABLE_AHRS
                ChassisLG::set_mode(ChassisLG::GIMBAL_REF_MODE);
#else
                ChassisLG::set_mode(ChassisLG::CHASSIS_REF_MODE);
#endif
            }
        }

        if (key_flag & (1U << Remote::KEY_X)) {
            if (Remote::key.ctrl && Remote::key.shift) {
                // Ctrl + Shift + X: ignore shoot constraints
                ignore_shoot_constraints = true;
            } else {
                // X: chassis dodge mode
#if ENABLE_AHRS
                ChassisLG::set_mode(ChassisLG::DODGE);
#else
                ChassisLG::set_mode(ChassisLG::CHASSIS_REF_MODE);
#endif
            }
        }

        if (key_flag & (1U << Remote::KEY_Q)) {
            // Q: left rotate 90 degree
            gimbal_yaw_target_angle_ += 90.0f;
            GimbalLG::set_target_angle(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_);
        } else if (key_flag & (1U << Remote::KEY_E)) {
            // E: left rotate 90 degree
            gimbal_yaw_target_angle_ -= 90.0f;
            GimbalLG::set_target_angle(gimbal_yaw_target_angle_, gimbal_pc_pitch_target_angle_);
        }
    }
}