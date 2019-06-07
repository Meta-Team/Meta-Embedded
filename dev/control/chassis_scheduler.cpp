//
// Created by Administrator on 2019/1/11 0011.
//

#include "chassis_scheduler.h"

PIDController Chassis::pid[Chassis::MOTOR_COUNT];
float Chassis::target_velocity[Chassis::MOTOR_COUNT];
float Chassis::w_to_v_ratio_ = 0.0f;
float Chassis::v_to_wheel_angular_velocity_ = 0.0f;

void Chassis::init(CANInterface *can_interface, float wheel_base, float wheel_tread, float wheel_circumference) {
    ChassisIF::init(can_interface);
    w_to_v_ratio_ = (wheel_base + wheel_tread) / 2.0f / 360.0f * 3.14159f;
    v_to_wheel_angular_velocity_ = (360.0f / wheel_circumference);
}

void Chassis::change_pid_params(PIDControllerBase::pid_params_t pid_params) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        pid[i].change_parameters(pid_params);
    }
}

void Chassis::calc(float target_vx, float target_vy, float target_w) {

    // FR, -vx, +vy, +w
    // FL, -vx, -vy, +w, since the motor is installed in the opposite direction
    // BL, +vx, -vy, +w, since the motor is installed in the opposite direction
    // BR, +vx, +vy, +w

    target_velocity[FR] = (-target_vx + target_vy + target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[FR] = (int) pid[FR].calc(feedback[FR].actual_velocity,
                                                            target_velocity[FR]);
    target_velocity[FL] = (-target_vx - target_vy + target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[FL] = (int) pid[FL].calc(feedback[FL].actual_velocity,
                                                            target_velocity[FL]);
    target_velocity[BL] = (+target_vx - target_vy + target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[BL] = (int) pid[BL].calc(feedback[BL].actual_velocity,
                                                            target_velocity[BL]);
    target_velocity[BR] = (+target_vx + target_vy + target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[BR] = (int) pid[BR].calc(feedback[BR].actual_velocity,
                                                            target_velocity[BR]);
}

