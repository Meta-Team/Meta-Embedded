//
// Created by liuzikai on 2019-01-05.
//

#include "gimbal_controller.h"

GimbalController::MotorController GimbalController::yaw(GimbalController::YAW_ID);
GimbalController::MotorController GimbalController::pitch(GimbalController::PIT_ID);
GimbalController::MotorController GimbalController::bullet_loader(GimbalController::BULLET_LOADER_ID);
float GimbalController::shoot_duty_cycles[4] = {0.0, 0.1, 0.2, 0.3};  // the numbers are temporary
int GimbalController::remained_bullet = 0;


/**
 * Public Functions
 */

/*
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
*/

int GimbalController::get_remained_bullet() {
    return remained_bullet;
}

void GimbalController::update_bullet(int bullet_changed) {
    remained_bullet += bullet_changed;
    if(remained_bullet < 0){
        remained_bullet = 0;
    }
}

/**
 * Private Functions
 */
