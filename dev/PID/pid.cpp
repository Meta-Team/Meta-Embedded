//
// Created by Administrator on 2018/12/1 0001.
//

#include "pid.h"

/**
 * Change parameters of the pidUnit of a pid_controller without cleaning current state
 * @param pid
 * @param kp
 * @param ki
 * @param kd
 * @param i_limit
 * @param out_limit
 */
void pid_controller::pid_change(float kp, float ki, float kd, float i_limit, float out_limit) {
    this->pidUnit.kp = kp;
    this->pidUnit.ki = ki;
    this->pidUnit.kd = kd;

    this->pidUnit.i_limit = i_limit;
    this->pidUnit.out_limit = out_limit;
}

/**
 * Perform a basic pid calculation
 * @param pid
 * @param now
 * @param target
 * @return the calculation result
 */
float pid_controller::pid_calc(float now, float target) {
    this->pidUnit.error[1] = this->pidUnit.error[0];
    this->pidUnit.error[0] = target - now;

    this->pidUnit.p_out = this->pidUnit.kp * this->pidUnit.error[0];
    this->pidUnit.i_out += this->pidUnit.ki * this->pidUnit.error[0];
    this->pidUnit.d_out = this->pidUnit.kd * (this->pidUnit.error[0] - this->pidUnit.error[1]);

    this->pidUnit.i_out = this->pidUnit.i_out > this->pidUnit.i_limit?this->pidUnit.i_limit:this->pidUnit.i_out;
    this->pidUnit.out = this->pidUnit.p_out + this->pidUnit.i_out + this->pidUnit.d_out;
    this->pidUnit.out = this->pidUnit.out > this->pidUnit.out_limit?this->pidUnit.out_limit:this->pidUnit.out;

    return this->pidUnit.out;
}

/*
     * Func:  initialize position-current pid control for gimbal control
     * Param: motor_id : gimbal motor_id
     *        pid_model1: the outer Position-velocity loop, 1 for fully PID, 0 for P & I only
     *        pid_model2: the inner velocity-current loop, 1 for fully PID, 0 for P & I only
     *        kp1: the outer Position-velocity loop kp
     *        ki1: the outer Position-velocity loop ki
     *        i_limit1: the outer Position-velocity loop maximum
     *        out_limit1: the outer Position-velocity loop output current maximum
     *        kp2: the inner Position-velocity loop kp
     *        ki2: the inner Position-velocity loop ki
     *        i_limit2: the inner Position-velocity loop maximum
     *        out_limit2: the inner Position-velocity loop output current maximum
     */
motor_pid::motor_pid(int motor_id, int pid_mode1, int pid_mode2, float kp1, float ki1, float kd1,
                     float i_limit1, float out_limit1, float kp2, float ki2, float kd2, float i_limit2,
                     float out_limit2) {
    this->motor_id = motor_id;
    this->pos_to_v_pid = new pid_controller(motor_id, pid_mode1, kp1, ki1, kd1, i_limit1, out_limit1);
    this->v_to_i_pid = new pid_controller(motor_id, pid_mode2, kp2, ki2, kd2, i_limit2, out_limit2);
}

/*
     * Func:  change parameter for a motor PID controller
     * Param:
     *        kp1: the outer Position-velocity loop kp
     *        ki1: the outer Position-velocity loop ki
     *        i_limit1: the outer Position-velocity loop maximum
     *        out_limit1: the outer Position-velocity loop output current maximum
     *        kp2: the inner Position-velocity loop kp
     *        ki2: the inner Position-velocity loop ki
     *        i_limit2: the inner Position-velocity loop maximum
     *        out_limit2: the inner Position-velocity loop output current maximum
     */
void motor_pid::motor_pid_change_Params(float kp1, float ki1, float kd1, float i_limit1, float out_limit1, float kp2,
                                       float ki2, float kd2, float i_limit2, float out_limit2) {
    this->pos_to_v_pid->pid_change(kp1, ki1, kd1, i_limit1, out_limit1);
    this->v_to_i_pid->pid_change(kp2, ki2, kd2, i_limit2, out_limit2);
}

/*
     * Func: fetch current calculated by PID controller at each time step
     * Param: measured_pos: actual position measured by the gyroscope
     *        measured_velocity: actual angular velocity of the motor, measured by gyroscope or calculated from the position (depending on time interval)
     */
float motor_pid::motor_pid_fetch_result(float measured_pos, float measured_velocity, float target_pos) {
    float target_velocity = this->pos_to_v_pid->pid_calc(measured_pos, target_pos);
    float target_current = this->v_to_i_pid->pid_calc(measured_velocity, target_velocity);
    return target_current;
}

/*
     * Func: free motor_pid at the end of your program
     */
void motor_pid::motor_pid_destroy() {
    delete this->v_to_i_pid;
    delete this->pos_to_v_pid;
}

