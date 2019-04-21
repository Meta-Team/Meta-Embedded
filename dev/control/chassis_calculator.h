//
// Created by Ye Anbang on 2019/1/11 0011.
// Rebuild by liuzikai
//

#ifndef META_INFANTRY_CHASSIS_CONTROLLER_H
#define META_INFANTRY_CHASSIS_CONTROLLER_H

#include "pid_controller.h"
#include "chassis_common.h"

#define CHASSIS_WHEEL_BASE 545.0f // distance between front axle and the back axle, mm
#define CHASSIS_WHEEL_TREAD 588.0f // distance between left and right wheels, mm
#define CHASSIS_WHEEL_CIRCUMFERENCE 478.0f // mm

/**
 *
 * +X is right, +Y is front, +w is counter-clockwise
 * TODO: add comments
 */
class ChassisController {

private:

    // Angular velocity (degree/s) to velocity (mm/s, based on mechanism structure)
    static float constexpr w_to_v_ratio = (CHASSIS_WHEEL_BASE + CHASSIS_WHEEL_TREAD) / 2.0f / 360.0f * 3.14159f;

    // Wheel speed (mm/s) to wheel angular velocity (degree/s)
    static float constexpr v_to_wheel_angular_velocity = (360.0f / CHASSIS_WHEEL_CIRCUMFERENCE);

public:

    typedef struct {

        chassis_motor_id_t id;

        PIDController pid;

        float target_velocity;
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

    static void rotateAroundWheel(chassis_motor_id_t pivotWheel, bool isClockwise, float angular_velocity);

};

#endif //PROJECT_CHASSIS_CONTROLLER_H
