//
// Created by liuzikai on 2019-05-01.
//

#include "shoot.h"

PIDController Shoot::v2i_pid[2];

float Shoot::degree_per_bullet_;
float Shoot::degree_per_bullet_plate_;

void Shoot::init(CANInterface *can_interface, float degree_per_bullet, float degree_per_bullet_plate) {
    GimbalInterface::init(can_interface, 0, 0);
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
    /** NOTICE: minus sign has been added here */
    target_current[BULLET] = (int) v2i_pid[0].calc(feedback[BULLET].actual_velocity,
                                                -degree_per_bullet_ * bullet_per_second);
    target_current[PLATE] = (int) v2i_pid[1].calc(feedback[PLATE].actual_velocity,
                                                -degree_per_bullet_plate_ * bullet_per_second);
}

void Shoot::set_friction_wheels(float duty_cycle) {
    fw_duty_cycle = duty_cycle;
}
