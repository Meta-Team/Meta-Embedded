//
// Created by Administrator on 2019/1/11 0011.
//

#include "chassis.h"

PIDController Chassis::pid[Chassis::CHASSIS_MOTOR_COUNT];
float Chassis::target_velocity[Chassis::CHASSIS_MOTOR_COUNT];
float Chassis::w_to_v_ratio_ = 0.0f;
float Chassis::v_to_wheel_angular_velocity_ = 0.0f;

void Chassis::init(CANInterface *can_interface, float wheel_base, float wheel_tread, float wheel_circumference) {
    ChassisInterface::init(can_interface);
    w_to_v_ratio_ = (wheel_base + wheel_tread) / 2.0f / 360.0f * 3.14159f;
    v_to_wheel_angular_velocity_ = (360.0f / wheel_circumference);
}

void Chassis::change_pid_params(float kp_v_loop, float ki_v_loop, float kd_v_loop, float i_limit, float out_limit) {
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        pid[i].change_parameters(kp_v_loop, ki_v_loop, kd_v_loop, i_limit, out_limit);
    }
}

void Chassis::calc(float target_vx, float target_vy, float target_w) {

    // FR, -vx, +vy, +w
    // FL, -vx, -vy, +w, since the motor is installed in the opposite direction
    // BL, +vx, -vy, +w, since the motor is installed in the opposite direction
    // BR, +vx, +vy, +w

    target_velocity[CHASSIS_FR] = (-target_vx + target_vy + target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[CHASSIS_FR] = (int) pid[CHASSIS_FR].calc(feedback[CHASSIS_FR].actual_velocity,
                                                            target_velocity[CHASSIS_FR]);
    target_velocity[CHASSIS_FL] = (-target_vx - target_vy + target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[CHASSIS_FL] = (int) pid[CHASSIS_FL].calc(feedback[CHASSIS_FL].actual_velocity,
                                                            target_velocity[CHASSIS_FL]);
    target_velocity[CHASSIS_BL] = (+target_vx - target_vy + target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[CHASSIS_BL] = (int) pid[CHASSIS_BL].calc(feedback[CHASSIS_BL].actual_velocity,
                                                            target_velocity[CHASSIS_BL]);
    target_velocity[CHASSIS_BR] = (+target_vx + target_vy + target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[CHASSIS_BR] = (int) pid[CHASSIS_BR].calc(feedback[CHASSIS_BR].actual_velocity,
                                                            target_velocity[CHASSIS_BR]);
}