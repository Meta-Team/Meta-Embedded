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

void chasisPidController::chasis_pid_change_parameters(float kp_v_loop, float ki_v_loop, float kd_v_loop, float i_limit,
                                                      float out_limit) {
/*
 * Param:
 *      chasis_PIDController: chasis_PIDController structure pointer to change parameters
 *      kp_ang_w_loop: angular movement kp for angular velocity
 *      ki_ang_w_loop: angular movement ki for angular velocity
 *      kd_ang_w_loop: angular movement kd for angular velocity
 *      kp_v_loop: velocity loop kp
 *      ki_v_loop: velocity loop ki
 *      kd_v_loop: velocity loop kd
 *      w_limit: angular velocity limit
 *      out_limit: the output current limit
 *
 * Func:
 *      setup a new chasis_controller with following parameters
 *
 * Return
 *      the pointer to the new allocated chasis_controller
 */
    change_parameters(kp_v_loop, ki_v_loop, kd_v_loop, i_limit, out_limit);
}


void chasisPidController::chasis_pid_calc(float measured_velocity_X, float measured_velocity_Y,
                                                  float measured_ang, float target_velocity_X, float target_velocity_Y,
                                                  float target_angular_velocity) {
    /*
     * Param:
     *      chasisPidController: pointer to a chasis_PIDController structure
     *      measured_velocity_X: measured velocity along the x axis with respect to the front of the chasis
     *      measured_velocity_Y: measured velocity along the y axis with respect to the front of the chasis
     *      measured_ang: measured angular with respect to the front of the chasis ( form -180 to 180)
     *      target_velocity_X: target velocity along the x axis with respect to the front of the chasis
     *      target_velocity_Y: target velocity along the y axis with respect to the front of the chasis
     *      target_ang: target angular with respect to the front of the chasis
     */
    float d_ang;
    if ((measured_ang - chasis_ang) > max_ang * 0.7) {
        d_ang = measured_ang - chasis_ang - max_ang*2;
    }
    else if ((measured_ang - chasis_ang) < -max_ang * 0.7) {
        d_ang = 360 - measured_ang - chasis_ang;
    }
    else d_ang = measured_ang - chasis_ang;
    float real_ang_velocity = d_ang / max_ang * 180 / chasisl_thread_interval * 1000.0f;
    chasis_motor_I_front_left = calc(measured_velocity_Y + measured_velocity_X - real_ang_velocity * front_rotate_ratio,
                                                                target_velocity_Y + target_velocity_X - target_angular_velocity * front_rotate_ratio);

    chasis_motor_I_front_right = calc(measured_velocity_Y - measured_velocity_X + real_ang_velocity * front_rotate_ratio,
                                                                 target_velocity_Y - target_velocity_X + target_angular_velocity * front_rotate_ratio);

    chasis_motor_I_back_left = calc(measured_velocity_Y - measured_velocity_X  - real_ang_velocity * back_rotate_ratio,
                                                               target_velocity_Y - target_velocity_X - target_angular_velocity *back_rotate_ratio);

    chasis_motor_I_back_right = calc(measured_velocity_Y + measured_velocity_X + real_ang_velocity * back_rotate_ratio,
                                                                target_velocity_Y + target_velocity_X + target_angular_velocity * back_rotate_ratio);

    chasis_ang = measured_ang;

    read_all_current(output);
}

void chasisPidController::chasis_pid_set_front(float measured_ang) {
    front_ang = measured_ang;
}

float chasisPidController::read_front_ang() {
    return front_ang;
}

void chasisPidController::read_all_current(float *current_array) {
    current_array[0] = chasis_motor_I_front_left;    //target current for front left motor
    current_array[1] = chasis_motor_I_front_right;   //target current for front right motor
    current_array[2] = chasis_motor_I_back_left; //target current for back left motor
    current_array[3] = chasis_motor_I_back_right; //target current for back right motor
}
