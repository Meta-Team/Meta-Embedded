//
// Created by liuzikai on 2019-01-05.
//

#include "gimbal.h"

PIDController Gimbal::a2v_pid[MOTOR_COUNT];
PIDController Gimbal::v2i_pid[MOTOR_COUNT];
float Gimbal::target_velocity[MOTOR_COUNT];

void Gimbal::init(CANInterface *can_interface, uint16_t yaw_front_angle_raw, uint16_t pitch_front_angle_raw,
                  PIDControllerBase::pid_params_t yaw_a2v_params, PIDControllerBase::pid_params_t yaw_v2i_params,
                  PIDControllerBase::pid_params_t pitch_a2v_params, PIDControllerBase::pid_params_t pitch_v2i_params) {

    GimbalInterface::init(can_interface, yaw_front_angle_raw, pitch_front_angle_raw);

    a2v_pid[YAW].change_parameters(yaw_a2v_params);
    v2i_pid[YAW].change_parameters(yaw_v2i_params);

    a2v_pid[PITCH].change_parameters(pitch_a2v_params);
    v2i_pid[PITCH].change_parameters(pitch_v2i_params);

}

void Gimbal::calc_a2v_(GimbalInterface::motor_id_t id_, float actual_angle_, float target_angle_) {
    target_velocity[id_] = a2v_pid[id_].calc(actual_angle_, target_angle_);
}

void Gimbal::calc_v2i_(GimbalInterface::motor_id_t id_, float actual_velocity_, float target_velocity_) {
    target_velocity[id_] = target_velocity_;
    target_current[id_] = (int) v2i_pid[id_].calc(actual_velocity_, target_velocity_);
}

void Gimbal::calc_gimbal(float yaw_actual_velocity, float pitch_actual_velocity,
                         float yaw_target_angle, float pitch_target_angle) {

    calc_a2v_(YAW, feedback[YAW].actual_angle, yaw_target_angle);
    calc_v2i_(YAW, yaw_actual_velocity, target_velocity[YAW]);

    calc_a2v_(PITCH, feedback[PITCH].actual_angle, pitch_target_angle);
    calc_v2i_(PITCH, pitch_actual_velocity, target_velocity[PITCH]);

}

GimbalController::BulletLoaderController GimbalController::bullet_loader(GimbalController::BULLET_LOADER_ID);
int GimbalController::remained_bullet = 0;


/**
 * Public Functions
 */

float GimbalController::MotorController::angle_to_v(float measured_angle, float target_angle) {
    return angle_to_v_pid.calc(measured_angle, target_angle);
}

float GimbalController::MotorController::v_to_i(float measured_velocity, float target_velocity) {
    return v_to_i_pid.calc(measured_velocity, target_velocity);
}

int GimbalController::BulletLoaderController::get_remained_bullet() {
    return remained_bullet;
}

void GimbalController::BulletLoaderController::update_bullet(int bullet_changed) {
    remained_bullet += bullet_changed;
    if (remained_bullet < 0) {
        remained_bullet = 0;
    }
}

void GimbalController::BulletLoaderController::start_continuous_shooting() {
    if (shooting) stop_shooting();
    continuous_shooting = true;
    shooting = true;
    shoot_accumulate_angle = 0;
}

void GimbalController::BulletLoaderController::start_incontinuous_shooting(int bullet_num) {
    if (shooting) stop_shooting();
    shoot_target_angle = one_bullet_step * bullet_num;
    continuous_shooting = false;
    shooting = true;
    shoot_accumulate_angle = 0;
}

void GimbalController::BulletLoaderController::stop_shooting() {
    shooting = false;
    int bullet_shot = (int) (shoot_accumulate_angle / one_bullet_step);  // calculate the number of the shot bullets
    update_bullet(-bullet_shot);  // update the number of remained bullets
}

void GimbalController::BulletLoaderController::update_accumulation_angle(float accumulate_angle) {
    if (shooting) {
        shoot_accumulate_angle += accumulate_angle - last_accumulate_angle;
        if (!continuous_shooting && shoot_accumulate_angle > shoot_target_angle) {
            stop_shooting();
        }
    }
    last_accumulate_angle = accumulate_angle;
}

float GimbalController::BulletLoaderController::get_target_current(float measured_velocity, float target_velocity) {
    if (shooting) {
        return v_to_i_pid.calc(measured_velocity, target_velocity);
    } else {
        return v_to_i_pid.calc(measured_velocity, 0);
    }
}

bool GimbalController::BulletLoaderController::get_shooting_status() {
    return shooting;
}

/**
 * Private Functions
 */
