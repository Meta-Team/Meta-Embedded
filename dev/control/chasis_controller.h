//
// Created by Ye Anbang on 2019/1/11 0011.
//

#ifndef PROJECT_CHASIS_CONTROLLER_H
#define PROJECT_CHASIS_CONTROLLER_H
#include "pid_controller.h"

#define MY_INFINITY 10000.0

/********** Chassis Structure Parameters **********/
#define CHASSIS_WHEEL_PERIMETER  478 //mm

/* the deceleration ratio of chassis motor */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)


float const chasisl_thread_interval = 50; // ms

float const maximum_current = 4000; // mA

float const max_ang = 180;    //max angle read by the electronic speed controller (Please change it for your convenience!!)

float const front_wheel_x = 0; //Please change this value

float const front_wheel_y = 0; //Please change this value

float const behind_wheel_x = 0; //Please change this value

float const behind_wheel_y = 0; //Please change this value

float const chasis_i_limit = 3000.0f;

float const chasis_max_ang_velocity = 1; //degree per ms


class chasisPidController: public PIDController{
private:
    float *output;
    float front_ang;
    float chasis_ang;
    float front_rotate_ratio;
    float back_rotate_ratio;
    float current_ratio;
    float chasis_motor_I_front_left;    //target current for front left motor
    float chasis_motor_I_front_right;   //target current for front right motor
    float chasis_motor_I_back_left; //target current for back left motor
    float chasis_motor_I_back_right;    //target current for back right motor

public:
    chasisPidController():PIDController() {}

    chasisPidController(float* output_array):PIDController(){
        output = output_array;
    }

    chasisPidController(float* output_array, float kp_v_loop, float ki_v_loop, float kd_v_loop, float i_limti,
                        float out_limit, float current_ang): PIDController(kp_v_loop, ki_v_loop, kd_v_loop, i_limti, out_limit) {
        /*
 * Param:
 *      kp_ang_w_loop: angular movement kp for angular velocity
 *      ki_ang_w_loop: angular movement ki for angular velocity
 *      kd_ang_w_loop: angular movement kd for angular velocity
 *      kp_v_loop: velocity loop kp
 *      ki_v_loop: velocity loop ki
 *      kd_v_loop: velocity loop kd
 *      w_limit: angular velocity limit
 *      out_limit: the output current limit
 *      current_ang: current absolute ang when setup
 * Func:
 *      setup a new chasis_controller with following parameters*/
        output = output_array;
        front_rotate_ratio = front_wheel_x + front_wheel_y;
        back_rotate_ratio = behind_wheel_x + behind_wheel_y;
        current_ratio = (60.0f / (CHASSIS_WHEEL_PERIMETER * CHASSIS_DECELE_RATIO));
        chasis_motor_I_front_left = chasis_motor_I_front_right = chasis_motor_I_back_left = chasis_motor_I_back_right = 0.0f;
        chasis_pid_set_front(current_ang);
        chasis_ang = current_ang;
    }
    void chasis_pid_change_parameters(float kp_v_loop, float ki_v_loop, float kd_v_loop, float i_limit,float out_limit);
    void chasis_pid_calc(float measured_velocity_X, float measured_velocity_Y, float measured_ang, float target_velocity_X,
                                 float target_velocity_Y, float target_angular_velocity);
    void chasis_pid_set_front(float measured_ang);
    float read_front_ang();
    void read_all_current(float *current_array);
};

#endif //PROJECT_CHASIS_CONTROLLER_H
