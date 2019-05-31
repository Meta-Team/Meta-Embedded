//
// Created by liuzikai on 2019-05-01.
//

#include "shoot.h"

PIDController Shoot::v2i_pid;

float Shoot::degree_per_bullet_;

void Shoot::init(float degree_per_bullet) {
    degree_per_bullet_ = degree_per_bullet;
}

void Shoot::change_pid_params(PIDControllerBase::pid_params_t bullet_loader_v2i_params) {
    v2i_pid.change_parameters(bullet_loader_v2i_params);
}

void Shoot::calc_bullet_loader(float bullet_per_second) {
    /** NOTICE: minus sign has been added here */
    target_current[BULLET] = (int) v2i_pid.calc(feedback[BULLET].actual_velocity,
                                                degree_per_bullet_ * bullet_per_second);
}

void Shoot::set_friction_wheels(float duty_cycle) {
    fw_duty_cycle = duty_cycle;
}