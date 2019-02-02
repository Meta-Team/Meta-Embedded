//
// Created by liuzikai on 2019-01-05.
//

#include "gimbal_controller.h"

GimbalController::MotorController GimbalController::yaw(GimbalController::YAW_ID);
GimbalController::MotorController GimbalController::pitch(GimbalController::PIT_ID);
GimbalController::MotorController GimbalController::bullet_loader(GimbalController::BULLET_LOADER);
GimbalController::FrictionWheelController GimbalController::frictionWheelController;

