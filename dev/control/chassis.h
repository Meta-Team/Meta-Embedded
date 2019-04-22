//
// Rebuild by liuzikai
//

#ifndef META_INFANTRY_CHASSIS_CONTROLLER_H
#define META_INFANTRY_CHASSIS_CONTROLLER_H

#include "ch.hpp"
#include "hal.h"
#include "chassis_interface.h"
#include "pid_controller.hpp"


/**
 * @name Chassis
 * @brief
 * @note +X is right, +Y is front, +w is counter-clockwise
 */
class Chassis : public ChassisInterface {

public:

    /**
     * @brief initialize ChassisInterface and this calculator
     * @param can_interface
     * @param wheel_base
     * @param wheel_tread
     * @param wheel_circumference
     */
    static void init(CANInterface* can_interface, float wheel_base, float wheel_tread, float wheel_circumference);

    /**
     * @brief   change parameters of PID controller of each motor
     * @param   kp_v_loop: velocity loop kp
     * @param   ki_v_loop: velocity loop ki
     * @param   kd_v_loop: velocity loop kd
     * @param   i_limit: i limit
     * @param   out_limit: the output current limit
     */
    static void change_pid_params(float kp_v_loop, float ki_v_loop, float kd_v_loop, float i_limit, float out_limit);

    /**
     * @brief calculate current for all chassis motors and fill target_velocity[] (in this class) and
     *        target_current[] (in ChassisInterface)
     * @param target_vx: target velocity along the x axis with respect to the front of the chassis (mm/s)
     * @param target_vy: target velocity along the y axis with respect to the front of the chassis (mm/s)
     * @param target_w: target angular with respect to the front of the chassis (degree/s, negative value for clockwise)
     */
    static void calc(float target_vx, float target_vy, float target_w);

    /**
     * @brief PID controller for each motor
     * @note For debug use
     */
    static PIDController pid[CHASSIS_MOTOR_COUNT];

    /**
     * @brief target_velocity for each motor (mid value for two-ring PID)
     * @note For debug use
     */
    static float target_velocity[CHASSIS_MOTOR_COUNT];

private:

    // Angular velocity (degree/s) to velocity (mm/s, based on mechanism structure)
    static float w_to_v_ratio_;

    // Wheel speed (mm/s) to wheel angular velocity (degree/s)
    static float v_to_wheel_angular_velocity_;

};

#endif //PROJECT_CHASSIS_CONTROLLER_H
