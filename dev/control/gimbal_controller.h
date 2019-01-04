//
// Created by liuzikai on 2019-01-05.
//

#ifndef META_INFANTRY_GIMBAL_CONTROLLER_H
#define META_INFANTRY_GIMBAL_CONTROLLER_H

#include "pid_controller.h"

class GimbalController {

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


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H
