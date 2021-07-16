//
// Created by Administrator on 2019/1/11 0011.
//

/**
 * @file    chassis_scheduler.cpp
 * @brief   Scheduler to control chassis to meet the target, including a thread to invoke PID calculation in period.
 *
 * @addtogroup chassis
 * @{
 */

#include "chassis_scheduler.h"

ChassisSKD::mode_t ChassisSKD::mode = FORCED_RELAX_MODE;

float ChassisSKD::target_vx;
float ChassisSKD::target_vy;
float ChassisSKD::target_theta;

PIDController ChassisSKD::a2v_pid;
PIDController ChassisSKD::v2i_pid[MOTOR_COUNT];

float ChassisSKD::target_w;
float ChassisSKD::target_velocity[MOTOR_COUNT];
int ChassisSKD::target_current[MOTOR_COUNT];

float ChassisSKD::wheel_base_ = 0;
float ChassisSKD::wheel_tread_ = 0;
float ChassisSKD::wheel_circumference_ = 0;
float ChassisSKD::w_to_v_ratio_ = 0.0f;
float ChassisSKD::v_to_wheel_angular_velocity_ = 0.0f;
float ChassisSKD::chassis_gimbal_offset_ = 0.0f;

ChassisSKD::install_mode_t ChassisSKD::install_mode_ = POSITIVE;

ChassisSKD::SKDThread ChassisSKD::skd_thread;


void ChassisSKD::start(float wheel_base, float wheel_tread, float wheel_circumference, install_mode_t install_mode,
                       float chassis_gimbal_offset, tprio_t thread_prio) {

    wheel_base_ = wheel_base;
    wheel_tread_ = wheel_tread;
    wheel_circumference_ = wheel_circumference;

    /*
     * FIXME: in the following lines, it should be 180.0f instead of 360.0f. However, this revision will affect
     *        many aspects, including Theta2W PID parameters for chassis dodge mode. It should be fixed after
     *        season 2019.
     */

#if defined(HERO)
    w_to_v_ratio_ = (wheel_base + wheel_tread) / 2.0f / 360.0f * 3.14159f;
#else
    w_to_v_ratio_ = (wheel_base + wheel_tread) / 2.0f / 360.0f * 3.14159f;
#endif
    v_to_wheel_angular_velocity_ = (360.0f / wheel_circumference_);
    chassis_gimbal_offset_ = chassis_gimbal_offset;
    install_mode_ = install_mode;

    skd_thread.start(thread_prio);
}

PIDController::pid_params_t ChassisSKD::echo_pid_params_by_type(bool is_theta2v) {
    return (is_theta2v) ? a2v_pid.get_parameters() : v2i_pid->get_parameters();
}

void ChassisSKD::load_pid_params(PIDControllerBase::pid_params_t theta2v_pid_params,
                                 PIDControllerBase::pid_params_t v2i_pid_params) {
    a2v_pid.change_parameters(theta2v_pid_params);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        v2i_pid[i].change_parameters(v2i_pid_params);
    }
}

void ChassisSKD::load_pid_params_by_type(PIDControllerBase::pid_params_t params, bool is_theta2v) {
    if (is_theta2v) {
        a2v_pid.change_parameters(params);
    } else {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            v2i_pid[i].change_parameters(params);
        }
    }
}

void ChassisSKD::set_mode(ChassisSKD::mode_t skd_mode) {
    mode = skd_mode;
}

void ChassisSKD::set_target(float vx, float vy, float theta) {
    target_vx = vx;
    target_vy = vy;
    target_theta = theta;
}

void ChassisSKD::set_dodge_target(float vx, float vy, float omega) {
    target_vx = vx;
    target_vy = vy;
    target_w = omega;
}

float ChassisSKD::get_actual_theta() {
    return GimbalIF::feedback[GimbalIF::YAW]->actual_angle * GIMBAL_YAW_INSTALL_DIRECTION;
}

float ChassisSKD::get_target_theta() {
    return target_theta;
}

void ChassisSKD::velocity_decompose_(float vx, float vy, float w) {

    // FR, +vx, -vy, +w
    // FL, +vx, +vy, +w, since the motor is installed in the opposite direction
    // BL, -vx, +vy, +w, since the motor is installed in the opposite direction
    // BR, -vx, -vy, +w

    target_velocity[FR] = install_mode_ * (+vx - vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[FR] = (int) v2i_pid[FR].calc(ChassisIF::feedback[FR]->actual_velocity, target_velocity[FR]);

    target_velocity[FL] = install_mode_ * (+vx + vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[FL] = (int) v2i_pid[FL].calc(ChassisIF::feedback[FL]->actual_velocity, target_velocity[FL]);

    target_velocity[BL] = install_mode_ * (-vx + vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[BL] = (int) v2i_pid[BL].calc(ChassisIF::feedback[BL]->actual_velocity, target_velocity[BL]);

    target_velocity[BR] = install_mode_ * (-vx - vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[BR] = (int) v2i_pid[BR].calc(ChassisIF::feedback[BR]->actual_velocity, target_velocity[BR]);
}

float ChassisSKD::get_actual_velocity(ChassisBase::motor_id_t motor_id) {
    return ChassisIF::feedback[motor_id]->actual_velocity;
}

void ChassisSKD::SKDThread::main() {
    setName("ChassisSKD");
    while (!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            if ((mode == GIMBAL_COORDINATE_MODE) || (mode == ANGULAR_VELOCITY_DODGE_MODE)) {

                float theta = get_actual_theta();

                if (mode == GIMBAL_COORDINATE_MODE) {
                    if (ABS(theta - target_theta) < THETA_DEAD_ZONE) {
                        target_w = 0;
                    } else {
                        target_w = a2v_pid.calc(theta, target_theta);
                    }
                }

                velocity_decompose(target_vx * cosf(theta / 180.0f * M_PI) - target_vy * sinf(theta / 180.0f * M_PI)
                                   - target_w / 180.0f * M_PI * chassis_gimbal_offset_,

                                   target_vx * sinf(theta / 180.0f * M_PI) + target_vy * cosf(theta / 180.0f * M_PI),

                                   target_w);

            } else if (mode == FORCED_RELAX_MODE) {

                for (size_t i = 0; i < MOTOR_COUNT; i++) {
                    target_current[i] = 0;
                }

            }

            // Send currents
            for (size_t i = 0; i < MOTOR_COUNT; i++) {
                *ChassisIF::target_current[i] = target_current[i];
            }
            ChassisIF::clip_chassis_current();
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}

/** @} */