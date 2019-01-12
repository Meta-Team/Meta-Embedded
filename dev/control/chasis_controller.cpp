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

void chassisPidController::chassis_pid_change_parameters(float kp_v_loop, float ki_v_loop, float kd_v_loop, float i_limit,
                                                      float out_limit) {
/**
 * @brief   change chassis pid params
 * @param   chassis_PIDController: chassis_PIDController structure pointer to change parameters
 * @param   kp_angular_w_loop: angular movement kp for angular velocity
 * @param   ki_angular_w_loop: angular movement ki for angular velocity
 * @param   kd_angular_w_loop: angular movement kd for angular velocity
 * @param   kp_v_loop: velocity loop kp
 * @param   ki_v_loop: velocity loop ki
 * @param   kd_v_loop: velocity loop kd
 * @param   w_limit: angular velocity limit
 * @param   out_limit: the output current limit
 *
 */
    change_parameters(kp_v_loop, ki_v_loop, kd_v_loop, i_limit, out_limit);
}


void chassisPidController::chassis_pid_calc(float measured_velocity_X, float measured_velocity_Y,
                                                  float measured_angle, float target_velocity_X, float target_velocity_Y,
                                                  float target_angular_velocity) {
    /**
     * @brief   calculate current for all chassis motors
     * @param   measured_velocity_X: measured velocity along the x axis with respect to the front of the chassis (mm/s)
     * @param   measured_velocity_Y: measured velocity along the y axis with respect to the front of the chassis (mm/s)
     * @param   measured_angle: measured angular with respect to the front of the chassis ( form -max_angle to max_angle, negative value for clockwise rotation) (degree/s)
     * @param   target_velocity_X: target velocity along the x axis with respect to the front of the chassis (mm/s)
     * @param   target_velocity_Y: target velocity along the y axis with respect to the front of the chassis (mm/s)
     * @param   target_angular_velocity: target angular with respect to the front of the chassis (degree/s, negative value for clockwise rotation)
     */
    float real_angular_velocity = (measured_angle - chassis_angle)/ chassis_thread_interval * 1000.0f;
    output[0] = calc(measured_velocity_Y + measured_velocity_X - real_angular_velocity * rotate_ratio,
                                                                target_velocity_Y + target_velocity_X - target_angular_velocity * rotate_ratio)*front_left_current_ratio*CHASSIS_WHEEL_RATIO;

    output[1] = calc(measured_velocity_Y - measured_velocity_X + real_angular_velocity * rotate_ratio,
                                                                 target_velocity_Y - target_velocity_X + target_angular_velocity * rotate_ratio)*front_right_current_ratio*CHASSIS_WHEEL_RATIO;

    output[2] = calc(measured_velocity_Y - measured_velocity_X  - real_angular_velocity * rotate_ratio,
                                                               target_velocity_Y - target_velocity_X - target_angular_velocity * rotate_ratio)*back_left_current_ratio*CHASSIS_WHEEL_RATIO;

    output[3] = calc(measured_velocity_Y + measured_velocity_X + real_angular_velocity * rotate_ratio,
                                                                target_velocity_Y + target_velocity_X + target_angular_velocity * rotate_ratio)*back_right_current_ratio*CHASSIS_WHEEL_RATIO;

    chassis_angle = measured_angle;
}

void chassisPidController::chassis_pid_set_front(float measured_angle) {
    /**
     * @brief   set front ang
     */
    front_angle = measured_angle;
}

float chassisPidController::read_front_angle() {
    /**
     * @brief   read front ang
     */
    return front_angle;
}