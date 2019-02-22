//
// Created by liuzikai on 2019-01-05.
//

#include "gimbal_controller.h"

GimbalController::MotorController GimbalController::yaw(GimbalController::YAW_ID);
GimbalController::MotorController GimbalController::pitch(GimbalController::PIT_ID);
GimbalController::BulletLoaderController GimbalController::bullet_loader(GimbalController::BULLET_LOADER_ID);
float GimbalController::shoot_duty_cycles[4] = {0.0, 0.1, 0.2, 0.3};  // the numbers are temporary
int GimbalController::remained_bullet = 0;


/**
 * Public Functions
 */

float GimbalController::MotorController::angle_to_v(float measured_angle, float target_angle)  {
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
    if(remained_bullet < 0){
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

/**
 * Private Functions
 */
