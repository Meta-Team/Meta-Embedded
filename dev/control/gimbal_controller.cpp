//
// Created by liuzikai on 2019-01-05.
//

#include "gimbal_controller.h"

GimbalController::MotorController GimbalController::yaw(GimbalController::YAW_ID);
GimbalController::MotorController GimbalController::pitch(GimbalController::PIT_ID);
GimbalController::BulletLoaderController GimbalController::bullet_loader(GimbalController::BULLET_LOADER_ID);
GimbalController::FrictionWheelController GimbalController::frictionWheelController;
std::vector<float> GimbalController::shoot_trigger_duty_cycle = {0.1, 0.2, 0.3};  // the numbers are temporary

int GimbalController::BulletLoaderController::get_current(float measured_angle, float measured_velocity,
                                                            float target_angle) {
    if(measured_angle<target_angle){
        float new_target_velocity = BulletLoaderController::angle_to_v_pid.calc(measured_angle,target_angle);
        return (int)BulletLoaderController::v_to_i_pid.calc(measured_velocity, new_target_velocity);
    } else{
        shooting_accomplished = true;
        return 0;
    }
}