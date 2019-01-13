//
// Created by Ye Anbang on 2019/1/11 0011.
//

#ifndef META_INFANTRY_CHASSIS_CONTROLLER_H
#define META_INFANTRY_CHASSIS_CONTROLLER_H

#include "pid_controller.h"
#include "chassis_common.h"

/********** Chassis Structure Parameters **********/
float const CHASSIS_WHEEL_RATIO = 478.0f / 2.0f; //mm
\

float const chassis_thread_interval = 50; // ms

float const maximum_current = 4000; // mA

/**
 * parameters below should be based on experiment value
 * for each motor, XX_XX_current_ratio = angular velocity change / current change
 */
float const front_left_current_ratio = 0.0f;  // TODO: Please change this value
float const front_right_current_ratio = 0.0f;   // TODO: Please change this value
float const back_left_current_ratio = 0.0f;     // TODO: Please change this value
float const back_right_current_ratio = 0.0f;    // TODO: Please change this value

/**
 *
 * +X is right, +Y is front, +w is counter-clockwise
 */
class ChassisController {

private:

    static float constexpr chassis_wheel_base = 420.0f; // distance between front axle and the back axle, mm
    static float constexpr chassis_wheel_tread = 370.0f; // distance between left and right wheels, mm
    static float constexpr chassis_wheel_circumference = 478.0f; // mm

    // Angular velocity (degree/s) to velocity (mm/s, based on mechanism structure)
    static float constexpr w_to_v_ratio = (chassis_wheel_base + chassis_wheel_tread) / 2.0f / 360.0f * 3.14159f;

    // Wheel speed (mm/s) to wheel angular velocity (degree/s)
    static float constexpr v_to_wheel_angular_velocity = (360.0f / chassis_wheel_circumference);

public:

    typedef struct {

        chassis_motor_id_t id;

        PIDController pid;

        float target_current;

    } motor_t;

    static motor_t motor[CHASSIS_MOTOR_COUNT]; // id is the same as those defined in chassis_motor_id_t

    /**
     * @brief   change parameters of PID controller of each motor
     * @param   kp_v_loop: velocity loop kp
     * @param   ki_v_loop: velocity loop ki
     * @param   kd_v_loop: velocity loop kd
     * @param   i_limit: i limit
     * @param   out_limit: the output current limit
     */
    static void change_pid_params(float kp_v_loop, float ki_v_loop, float kd_v_loop, float i_limit, float out_limit) {

        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
            motor[i].pid.change_parameters(kp_v_loop, ki_v_loop, kd_v_loop, i_limit, out_limit);
        }
    }

    /**
     * @brief calculate current for all chassis motors
     * @param measured_angular_velocity: measured motor velocity (degree/s)
     * @param target_vx: target velocity along the x axis with respect to the front of the chassis (mm/s)
     * @param target_vy: target velocity along the y axis with respect to the front of the chassis (mm/s)
     * @param target_w: target angular with respect to the front of the chassis (degree/s, negative value for clockwise)
     */
    static void calc(float measured_angular_velocity[CHASSIS_MOTOR_COUNT], float target_vx, float target_vy, float target_w);

};

#endif //PROJECT_CHASSIS_CONTROLLER_H
