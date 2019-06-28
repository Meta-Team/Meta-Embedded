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
 * @brief Chassis controller from high level to low level (by inheritance)
 * @note +X is right, +Y is front, +w is counter-clockwise
 */
class ChassisSKD : public ChassisIF, public PIDControllerBase {

public:

    /**
     * Initialize ChassisInterface and this calculator
     * @param can_interface
     * @param wheel_base
     * @param wheel_tread
     * @param wheel_circumference
     */
    static void init(CANInterface* can_interface, float wheel_base, float wheel_tread, float wheel_circumference);

    /**
     * Change parameters of PID controller of every motor
     * @param pid_params
     */
    static void change_pid_params(pid_params_t pid_params);

    /**
     * Calculate current for all chassis motors and fill target_velocity[] (in this class) and target_current[]
     * (in ChassisInterface)
     * @param target_vx: target velocity along the x axis with respect to the front of the chassis (mm/s)
     * @param target_vy: target velocity along the y axis with respect to the front of the chassis (mm/s)
     * @param target_w: target angular with respect to the front of the chassis (degree/s, negative value for clockwise)
     */
    static void calc(float target_vx, float target_vy, float target_w);

    /**
     * PID controller for each motor
     * @note For debug use
     */
    static PIDController pid[MOTOR_COUNT];

    /**
     * Target_velocity for each motor (mid value for two-ring PID)
     * @note For debug use
     */
    static float target_velocity[MOTOR_COUNT];

private:

    // Angular velocity (degree/s) to velocity (mm/s, based on mechanism structure)
    static float w_to_v_ratio_;

    // Wheel speed (mm/s) to wheel angular velocity (degree/s)
    static float v_to_wheel_angular_velocity_;

};

#endif //PROJECT_CHASSIS_CONTROLLER_H
