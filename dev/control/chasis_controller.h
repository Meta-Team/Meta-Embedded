//
// Created by Ye Anbang on 2019/1/11 0011.
//

#ifndef META_INFANTRY_CHASSIS_CONTROLLER_H
#define META_INFANTRY_CHASSIS_CONTROLLER_H

#include "pid_controller.h"
#include "chassis_common.h"

/********** Chassis Structure Parameters **********/
float const CHASSIS_WHEEL_RATIO = 478.0f / 2.0f; //mm
/*  */
float const CHASSIS_WHEELTRACK = 358; // wheel track distance (mm)
/* wheelbase distance (mm) */
float const CHASSIS_WHEELBASE = 358; // wheelbase distance (mm)

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
class ChassisController : public PIDController {

private:

    float const chassis_wheelbase = ???; // distance between front axle and the back axle, mm
    float const chassis_wheeltread = ???; // distance between left and right wheels, mm

    float front_angle;
    float chassis_angle;  // chassis angle with respect to front direction
    float rotate_ratio;

public:

    float target_current[CHASSIS_MOTOR_COUNT]; // id is the same as those defined in chassis_motor_id_t

    ChassisController() : PIDController() {}

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
    ChassisController(float kp_v_loop, float ki_v_loop, float kd_v_loop, float i_limti,
                      float out_limit, float current_angle) : PIDController(kp_v_loop, ki_v_loop, kd_v_loop, i_limti,
                                                                            out_limit) {

        rotate_ratio = (CHASSIS_WHEELBASE + CHASSIS_WHEELTRACK) / 2.0f;
        chassis_pid_set_front(current_angle);
        chassis_angle = current_angle;
    }

    /**
     * @brief   calculate current for all chassis motors
     * @param   measured_velocity_X: measured velocity along the x axis with respect to the front of the chassis (mm/s)
     * @param   measured_velocity_Y: measured velocity along the y axis with respect to the front of the chassis (mm/s)
     * @param   measured_angle: measured angular with respect to the front of the chassis ( form -max_angle to max_angle, negative value for clockwise rotation) (degree/s)
     * @param   target_velocity_X: target velocity along the x axis with respect to the front of the chassis (mm/s)
     * @param   target_velocity_Y: target velocity along the y axis with respect to the front of the chassis (mm/s)
     * @param   target_angular_velocity: target angular with respect to the front of the chassis (degree/s, negative value for clockwise rotation)
     */
    void chassis_pid_calc(float measured_velocity_X, float measured_velocity_Y, float measured_angle,
                          float target_velocity_X,
                          float target_velocity_Y, float target_angular_velocity);

    void chassis_pid_set_front(float measured_angle);

    float read_front_angle();
};

#endif //PROJECT_CHASSIS_CONTROLLER_H
