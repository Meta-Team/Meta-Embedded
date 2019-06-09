//
// Created by zhukerui on 2019/6/9.
//

#include "suspension_gimbal_controller.h"
#include "serial_shell.h"

float SuspensionGimbalController::shoot_duty_cycles[3] = {0.0, 0.1, 0.3};  // the numbers are temporary
int SuspensionGimbalController::remained_bullet = 0;


/**
 * Public Functions
 */

void SuspensionGimbalController::update_bullet(int bullet_changed) {
    remained_bullet += bullet_changed;
    if(remained_bullet < 0){
        remained_bullet = 0;
    }
}

void SuspensionGimbalController::start_continuous_shooting() {
    if (shooting) stop_shooting();
    continuous_shooting = true;
    shooting = true;
    shoot_accumulate_angle = 0;
}

void SuspensionGimbalController::start_incontinuous_shooting(int bullet_num) {
    if (shooting) stop_shooting();
    shoot_target_angle = one_bullet_step * bullet_num;
    continuous_shooting = false;
    shooting = true;
    shoot_accumulate_angle = 0;
}

void SuspensionGimbalController::stop_shooting() {
    shooting = false;
    int bullet_shot = (int) (shoot_accumulate_angle / one_bullet_step);  // calculate the number of the shot bullets
    update_bullet(-bullet_shot);  // update the number of remained bullets
}

void SuspensionGimbalController::update_accumulation_angle(float accumulate_angle) {
    if (shooting) {
        shoot_accumulate_angle += accumulate_angle - last_accumulate_angle;
        if (!continuous_shooting && shoot_accumulate_angle > shoot_target_angle) {
            stop_shooting();
        }
    }
    last_accumulate_angle = accumulate_angle;
}

float SuspensionGimbalController::get_target_current(float measured_velocity, float target_velocity) {
    if (shooting) {
        return BL_v_to_i.calc(measured_velocity, target_velocity);
    } else {
        return BL_v_to_i.calc(measured_velocity, 0);
    }
}