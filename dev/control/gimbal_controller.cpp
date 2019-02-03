//
// Created by liuzikai on 2019-01-05.
//

#include "gimbal_controller.h"

GimbalController::MotorController GimbalController::yaw(GimbalController::YAW_ID);
GimbalController::MotorController GimbalController::pitch(GimbalController::PIT_ID);
GimbalController::MotorController GimbalController::bullet_loader(GimbalController::BULLET_LOADER_ID);
GimbalController::FrictionWheelController GimbalController::frictionWheelController;

float GimbalController::get_bullet_loader_current(float measured_angle, float measured_velocity, float target_angle) {
    if(target_angle < measured_angle){
        /**
         * Since the bullet loader could only move towards positive direction,
         * if the target angle is smaller than the present angle, then we regard that the loader needs to finish a round
         */
        measured_angle -= 360.0f;
    }
    float target_velocity;
    target_velocity = bullet_loader.angle_to_v(measured_angle, target_angle);
    return bullet_loader.v_to_i(measured_velocity, target_velocity);
}

float GimbalController::get_gimbal_motor_current(float measured_angle, float measured_velocity, float target_angle) {}