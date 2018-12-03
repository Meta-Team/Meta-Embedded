//
// Created by Anbang Ye on 30/03/2018.
//

#ifndef INSOULED_CHIBIOS_PID_H
#define INSOULED_CHIBIOS_PID_H

//#include "../global.h"


typedef struct {
    float kp;
    float ki;
    float kd;

    float error[2];

    float p_out;
    float i_out;
    float d_out;

    float i_limit;
    float out_limit;

    float out;
} pid_unit;

enum pid_mode {
    PI, //PID only use P & I
    PID,    //fully PID
};

class pid_controller {
private:
    int motor_id;   //motor id
    int pid_mode;
    pid_unit pidUnit;   //info for pid control

public:
    pid_controller(int motor_id, int pid_mode, float kp, float ki, float kd, float i_limit, float out_limit) {
        this->motor_id = motor_id;
        this->pid_mode = pid_mode;
        pid_change(kp, ki, kd, i_limit, out_limit);
        this->pidUnit.p_out = this->pidUnit.i_out = this->pidUnit.d_out = this->pidUnit.out = 0.0;
        this->pidUnit.error[0] = this->pidUnit.error[1] = 0.0;
    }; //initialize pid_controller
    void pid_change(float kp, float ki, float kd, float i_limit, float out_limit);
    float pid_calc(float now, float target);
};

class motor_pid {
private:
    int motor_id;   //motor id
    pid_controller *pos_to_v_pid;
    pid_controller *v_to_i_pid;

public:
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
    motor_pid(int motor_id, int pid_mode1, int pid_mode2, float kp1, float ki1, float kd1, float i_limit1, float out_limit1,
              float kp2 = 0, float ki2 = 0, float kd2 = 0, float i_limit2 = 0, float out_limit2 = 0);

    /*
     * Func: change parameter for motor pid
     * Param: the same as the previous function
     */
    void motor_pid_change_Params(float kp1, float ki1, float kd1, float i_limit1, float out_limit1,
                                float kp2 = 0, float ki2 = 0, float kd2 = 0, float i_limit2 = 0, float out_limit2 = 0);

    /*
     * Func: fetch current calculated by PID controller at each time step
     * Param: measured_pos: actual position measured by the gyroscope
     *        measured_velocity: actual angular velocity of the motor, measured by gyroscope or calculated from the position (depending on time interval)
     */
    float motor_pid_fetch_result(float measured_pos, float measured_velocity, float target_pos);

    /*
     * Func: free motor_pid at the end of your program
     */
    void motor_pid_destroy();
};

typedef struct {
    motor_pid *motor_pid1;
    motor_pid *motor_pid2;
} gimbal_pid_controller;

#endif //INSOULED_CHIBIOS_PID_H