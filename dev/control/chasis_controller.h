//
// Created by Ye Anbang on 2019/1/11 0011.
//

#ifndef PROJECT_CHASSIS_CONTROLLER_H
#define PROJECT_CHASSIS_CONTROLLER_H
#include "pid_controller.h"

/********** Chassis Structure Parameters **********/
float const CHASSIS_WHEEL_RATIO = 478.0f/2.0f; //mm
/* wheel track distance(mm) */
float const CHASSIS_WHEELTRACK = 358;
/* wheelbase distance(mm) */
float const CHASSIS_WHEELBASE = 358;

float const chassis_thread_interval = 50; // ms

float const maximum_current = 4000; // mA

/**
 * parameters below should be based on experiment value
 * for each motor, XX_XX_current_ratio = angular velocity change / current change
 */
float const front_left_current_ratio = 0.0f;  //TODO: Please change this value

float const front_right_current_ratio = 0.0f;   //TODO: Please change this value

float const back_left_current_ratio = 0.0f;     //TODO: Please change this value

float const back_right_current_ratio = 0.0f;    //TODO: Please change this value


class chassisPidController: public PIDController{
private:
    float front_angle;    //[chasis_motor_I_front_left, chasis_motor_I_front_right, chasis_motor_I_front_left, chasis_motor_I_back_right]
    float chassis_angle;    //chasis_angle with respect to front direction
    float rotate_ratio;
public:
    float output[4];
    chassisPidController():PIDController() {}

    chassisPidController(float kp_v_loop, float ki_v_loop, float kd_v_loop, float i_limti,
                        float out_limit, float current_angle): PIDController(kp_v_loop, ki_v_loop, kd_v_loop, i_limti, out_limit) {
        /**
         * @brief   setup chassis controller
         * @param   kp_v_loop: velocity loop kp
         * @param   ki_v_loop: velocity loop ki
         * @param   kd_v_loop: velocity loop kd
         * @param   i_limit: i limit
         * @param   out_limit: the output current limit
         * @param   current_angle   current ang
         *
         */
        rotate_ratio = (CHASSIS_WHEELBASE + CHASSIS_WHEELTRACK)/2.0f;
        chassis_pid_set_front(current_angle);
        chassis_angle = current_angle;
    }
    void chassis_pid_change_parameters(float kp_v_loop, float ki_v_loop, float kd_v_loop, float i_limit,float out_limit);
    void chassis_pid_calc(float measured_velocity_X, float measured_velocity_Y, float measured_angle, float target_velocity_X,
                                 float target_velocity_Y, float target_angular_velocity);
    void chassis_pid_set_front(float measured_angle);
    float read_front_angle();
};

#endif //PROJECT_CHASIS_CONTROLLER_H
