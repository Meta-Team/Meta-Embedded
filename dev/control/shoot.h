//
// Created by liuzikai on 2019-05-01.
//

#ifndef META_INFANTRY_SHOOT_H
#define META_INFANTRY_SHOOT_H

#include "gimbal_interface.h"
#include "pid_controller.hpp"

/**
 * @name Shoot
 * @brief Shooter controller from high level to low level (by inheritance)
 * @note Share low-level GimbalInterface with Gimbal. Low-level initialization is done by Gimbal.
 */
class Shoot : public GimbalInterface, public PIDControllerBase {

public:

    /**
     * Initialize the shooter controller
     * @param degree_per_bullet
     * @param bullet_loader_v2i_params
     */
    static void init(CANInterface *can_interface, float degree_per_bullet, float degree_per_bullet_plate);

    /**
     * Change PID parameters of bullet loader
     * @param bullet_loader_v2i_params
     */
    static void change_pid_params(pid_params_t bullet_loader_v2i_params);
    static void change_plate_params(PIDControllerBase::pid_params_t bullet_plate_v2i_params);

    /**
     * Perform calculation from velocity to current and put result into target_current[]
     * @param bullet_per_second   shoot speed
     */
    static void calc_bullet_loader(float bullet_per_second);

    /**
     * Set friction wheel duty cycle
     * @param duty_cycle from 0 to 1
     */
    static void set_friction_wheels(float duty_cycle);

    static PIDController v2i_pid[2];

public:
    static float degree_per_bullet_;
    static float degree_per_bullet_plate_;

};


#endif //META_INFANTRY_SHOOT_H
