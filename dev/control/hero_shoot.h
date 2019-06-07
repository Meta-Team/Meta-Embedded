//
// Created by liuzikai on 2019-05-01.
//

#ifndef META_HERO_SHOOT_H
#define META_HERO_SHOOT_H

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
    static void init(float degree_per_bullet, float degree_per_bullet_plate);

    /**
     * Change PID parameters of bullet loader
     * @param bullet_loader_v2i_params
     */
    static void change_pid_params(PIDControllerBase::pid_params_t bullet_loader_a2v_params,PIDControllerBase::pid_params_t bullet_loader_v2i_params,
                                  PIDControllerBase::pid_params_t bullet_plate_a2v_params, PIDControllerBase::pid_params_t bullet_plate_v2i_params);

    /**
     * Calculate the current need to turn a angle
     * @param plate_target_angle, bullet_target_angle
     */


    /**
     * Set friction wheel duty cycle
     * @param duty_cycle from 0 to 1
     */
    static void set_friction_wheels(float duty_cycle);

    static PIDController v2i_pid[2];
    static PIDController a2v_pid[2];
    static void calc_shoot(float bullet_actual_velocity, float plate_actual_velocity,
                            float bullet_target_angle, float plate_target_angle);

private:

    static float degree_per_bullet_;
    static float degree_per_bullet_plate_;

    friend class ShootDebugThread;

    static float target_velocity[2];

    static void calc_a2v_(motor_id_t motor, float actual_angle_, float target_angle_);
    static void calc_v2i_(motor_id_t motor, float actual_velocity_, float target_velocity_);
};


#endif //META_INFANTRY_SHOOT_H
