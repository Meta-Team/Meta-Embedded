//
// Created by liuzikai on 2019-05-01.
//

#include "hero_shoot.h"

PIDController Shoot::v2i_pid[2];
PIDController Shoot::a2v_pid[2];
float Shoot::target_velocity[2];

float Shoot::degree_per_bullet_;
float Shoot::degree_per_bullet_plate_;

void Shoot::init(float degree_per_bullet, float degree_per_bullet_plate) {
    degree_per_bullet_ = degree_per_bullet;
    degree_per_bullet_plate_ = degree_per_bullet_plate;
}

void Shoot::change_pid_params(PIDControllerBase::pid_params_t bullet_loader_a2v_params,
                              PIDControllerBase::pid_params_t bullet_loader_v2i_params,
                              PIDControllerBase::pid_params_t bullet_plate_a2v_params,
                              PIDControllerBase::pid_params_t bullet_plate_v2i_params) {
    v2i_pid[0].change_parameters(bullet_loader_v2i_params);
    v2i_pid[1].change_parameters(bullet_plate_v2i_params);
    a2v_pid[0].change_parameters(bullet_loader_a2v_params);
    a2v_pid[1].change_parameters(bullet_plate_a2v_params);
}

void Shoot::calc_plate(float plate_actual_velocity, float plate_target_angle){
    if (plate_target_angle < degree_per_bullet_plate_ && feedback[PLATE].actual_angle > plate_actual_velocity + 360.0f - degree_per_bullet_plate_) // when here is definitely it has turned around.
    {
        plate_target_angle+=360.0f;
    }
    calc_a2v_(PLATE, feedback[PLATE].actual_angle, plate_target_angle);
    calc_v2i_(PLATE, plate_actual_velocity, target_velocity[PLATE-2]);
}

void Shoot::calc_bullet(float bullet_actual_velocity, float bullet_target_angle){
    if (bullet_target_angle < degree_per_bullet_ && feedback[BULLET].actual_angle > bullet_target_angle + 360.0f - degree_per_bullet_)
    {
        bullet_target_angle+=360.0f;
    }
    calc_a2v_(BULLET, feedback[BULLET].actual_angle, bullet_target_angle);
    calc_v2i_(BULLET, bullet_actual_velocity, target_velocity[BULLET-2]);
}

void Shoot::calc_a2v_(GimbalInterface::motor_id_t id_, float actual_angle_, float target_angle_) {
    target_velocity[id_-2] = a2v_pid[id_-2].calc(actual_angle_, target_angle_);
}

void Shoot::calc_v2i_(GimbalInterface::motor_id_t id_, float actual_velocity_, float target_velocity_) {
    target_velocity[id_-2] = target_velocity_;
    target_current[id_] = (int) v2i_pid[id_-2].calc(actual_velocity_, target_velocity_); // send currents to BULLET and PLATE
}

void Shoot::set_friction_wheels(float duty_cycle) {
    fw_duty_cycle = duty_cycle;
}
