//
// Created by liuzikai on 2019-01-05.
//

#include "gimbal_controller.h"

GimbalController::MotorController GimbalController::yaw(GimbalController::YAW_ID);
GimbalController::MotorController GimbalController::pitch(GimbalController::PIT_ID);
GimbalController::MotorController GimbalController::bullet_loader(GimbalController::BULLET_LOADER_ID);
GimbalController::FrictionWheelController GimbalController::frictionWheelController;
float GimbalController::shoot_trigger_duty_cycle[3] = {0.1, 0.2, 0.3};  // the numbers are temporary
float GimbalController::one_bullet_step;
int GimbalController::remained_bullet;
bool GimbalController::shooting;
float GimbalController::bullet_loader_velocity;


/**
 * Public Functions
 */

void GimbalController::start() {
    one_bullet_step = 40.0f;
    remained_bullet = 0;
    shooting = false;
    bullet_loader_velocity = 0.0;
}

bool
GimbalController::update_motor_data(GimbalController::motor_id_t motor_id, float actual_angle, float angular_velocity) {
    switch (motor_id){
        case BULLET_LOADER_ID:
            bullet_loader.actual_angle += actual_angle;
            bullet_loader.angular_velocity = angular_velocity;
            return true;
        case YAW_ID:
            yaw.actual_angle = actual_angle;
            yaw.angular_velocity = angular_velocity;
            return true;
        case PIT_ID:
            pitch.actual_angle = actual_angle;
            pitch.angular_velocity = angular_velocity;
            return true;
        default:
            return false;
    }
}

void GimbalController::shoot_bullet(GimbalController::shoot_mode_t shoot_mode, int bullet_num) {
    frictionWheelController.trigger_duty_cycle = frictionWheelController.actual_duty_cycle = shoot_trigger_duty_cycle[shoot_mode];
    bullet_loader.target_angle += one_bullet_step * bullet_num;
    shooting = true;
}

float GimbalController::get_fw_pid() {
    return frictionWheelController.trigger_duty_cycle;
}

int GimbalController::get_bullet_loader_target_current(){

    int target_current;
    float target_velocity;
    if(bullet_loader.actual_angle<bullet_loader.target_angle
    && (frictionWheelController.actual_duty_cycle>=frictionWheelController.trigger_duty_cycle)){
        // If the target angle is not reached and the speed of the friction wheels satisfies the requirement, we calculate the target velocity
        target_velocity = GimbalController::bullet_loader.angle_to_v(bullet_loader.actual_angle, bullet_loader.target_angle);
    } else
        // If the target angle is reached or the friction wheels don't satisfy the requirement, then the bullet loader should be stopped
        target_velocity = 0;

    target_current = (int)(GimbalController::bullet_loader.v_to_i(bullet_loader.angular_velocity, target_velocity));
    return target_current;
}

int GimbalController::update_bullet_count(int new_bullet_added) {
    if (new_bullet_added <= 0){
        // If it's not greater than 0, then it means we are now under the shooting mode
        int bullet_shot = (int)(bullet_loader.actual_angle/one_bullet_step);  // calculate the number of the shot bullets
        remained_bullet -= bullet_shot;  // update the number of remained bullets
        bullet_loader.target_angle -= (bullet_shot * one_bullet_step);
    } else{
        // If it's greater than 0, then it means we are now under the reloading mode
        remained_bullet += new_bullet_added;  // add the bullets
    }
    if(remained_bullet<0){
        remained_bullet = 0;  // if the number of remained bullets is negative, then regard it as 0
    }
    return remained_bullet;
}

int GimbalController::get_remained_bullet() {
    return remained_bullet;
}

void GimbalController::start_shooting() {
    bullet_loader.target_velocity = bullet_loader_velocity;
}

int GimbalController::get_current_temp() {
    return (int)bullet_loader.v_to_i(bullet_loader.angular_velocity, bullet_loader.target_velocity);
}

void GimbalController::stop_shooting() {
    bullet_loader.target_velocity = 0;
    GimbalController::update_bullet_count();
}

int GimbalController::load_bullet(int bullet_num) {
    return GimbalController::update_bullet_count(bullet_num);
}

/**
 * Private Functions
 */
