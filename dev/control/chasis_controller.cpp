//
// Created by Administrator on 2019/1/11 0011.
//

#include "chasis_controller.h"
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"

using namespace chibios_rt;

void ChassisController::chassis_pid_calc(float measured_velocity_X, float measured_velocity_Y,
                                         float measured_angle, float target_velocity_X, float target_velocity_Y,
                                         float target_angular_velocity) {
    float real_angular_velocity = (measured_angle - chassis_angle) / chassis_thread_interval * 1000.0f;

    target_current[CHASSIS_FR] = calc(measured_velocity_Y - measured_velocity_X + real_angular_velocity * rotate_ratio,
                     target_velocity_Y - target_velocity_X + target_angular_velocity * rotate_ratio) *
                front_right_current_ratio * CHASSIS_WHEEL_RATIO;

    target_current[CHASSIS_FL] = calc(-measured_velocity_Y - measured_velocity_X + real_angular_velocity * rotate_ratio,
                             -target_velocity_Y - target_velocity_X + target_angular_velocity * rotate_ratio) *
                        front_left_current_ratio * CHASSIS_WHEEL_RATIO;

    target_current[CHASSIS_BL] = calc(-measured_velocity_Y + measured_velocity_X + real_angular_velocity * rotate_ratio,
                     -target_velocity_Y + target_velocity_X + target_angular_velocity * rotate_ratio) *
                back_left_current_ratio * CHASSIS_WHEEL_RATIO;

    target_current[CHASSIS_BR] = calc(measured_velocity_Y + measured_velocity_X + real_angular_velocity * rotate_ratio,
                     target_velocity_Y + target_velocity_X + target_angular_velocity * rotate_ratio) *
                back_right_current_ratio * CHASSIS_WHEEL_RATIO;

    chassis_angle = measured_angle;
}

void ChassisController::chassis_pid_set_front(float measured_angle) {
    /**
     * @brief   set front ang
     */
    front_angle = measured_angle;
}

float ChassisController::read_front_angle() {
    /**
     * @brief   read front ang
     */
    return front_angle;
}