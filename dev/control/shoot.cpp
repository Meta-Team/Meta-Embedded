//
// Created by liuzikai on 2019-05-01.
//

#include "shoot.h"

PIDController Shoot::v2i_pid[2];

float Shoot::degree_per_bullet_;
float Shoot::degree_per_bullet_plate_;

void Shoot::init(float degree_per_bullet, float degree_per_bullet_plate) {
    degree_per_bullet_ = degree_per_bullet;
    degree_per_bullet_plate_ = degree_per_bullet_plate;
}

void Shoot::change_pid_params(PIDControllerBase::pid_params_t bullet_loader_v2i_params) {
    v2i_pid[0].change_parameters(bullet_loader_v2i_params);
}
void Shoot::change_plate_params(PIDControllerBase::pid_params_t bullet_plate_v2i_params) {
    v2i_pid[1].change_parameters(bullet_plate_v2i_params);
}

void Shoot::calc_bullet_loader(float bullet_per_second) {
    calc_motor_(BULLET, feedback[BULLET].actual_velocity, degree_per_bullet_ * bullet_per_second);
    calc_motor_(PLATE, feedback[PLATE].actual_velocity, degree_per_bullet_plate_ * bullet_per_second);
}

void Shoot::calc_motor_(GimbalInterface::motor_id_t motor, float actual_velocity, float target_velocity) {
    target_current[motor] = (int) v2i_pid[motor - 2].calc(actual_velocity, target_velocity);
}

void Shoot::set_friction_wheels(float duty_cycle) {
    fw_duty_cycle = duty_cycle;
}
