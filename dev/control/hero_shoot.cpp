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

void Shoot::change_pid_params(PIDControllerBase::pid_params_t bullet_loader_v2i_params,
                              PIDControllerBase::pid_params_t bullet_loader_a2v_params,
                              PIDControllerBase::pid_params_t bullet_plate_v2i_params,
                              PIDControllerBase::pid_params_t bullet_plate_a2v_params) {
    v2i_pid[0].change_parameters(bullet_loader_v2i_params);
    v2i_pid[1].change_parameters(bullet_plate_v2i_params);
    a2v_pid[0].change_parameters(bullet_loader_a2v_params);
    a2v_pid[1].change_parameters(bullet_plate_a2v_params);
}
void Shoot::calc_shoot(float bullet_actual_velocity, float plate_actual_velocity,
                       float bullet_target_angle, float plate_target_angle){
    if (plate_target_angle < degree_per_bullet_plate_ && feedback[PLATE].actual_angle > 360.0f - degree_per_bullet_plate_) // when here is definitely it has turned around.
    {
        plate_target_angle+=360.0f;
    }
    calc_a2v_(PLATE, feedback[PLATE].actual_angle, plate_target_angle);
    calc_v2i_(PLATE, plate_actual_velocity, target_velocity[PLATE]);

    if (bullet_target_angle < degree_per_bullet_ && feedback[BULLET].actual_angle > 360.0f - degree_per_bullet_)
    {
        bullet_target_angle+=360.0f;
    }
    calc_a2v_(BULLET, feedback[BULLET].actual_angle, bullet_target_angle);
    calc_v2i_(BULLET, bullet_actual_velocity, target_velocity[BULLET]);
}
void Shoot::calc_a2v_(GimbalInterface::motor_id_t motor, float actual_angle_, float target_angle_) {
    target_velocity[motor] = a2v_pid[motor].calc(actual_angle_, target_angle_);
}

void Shoot::calc_v2i_(GimbalInterface::motor_id_t motor, float actual_velocity_, float target_velocity_) {
    target_velocity[motor] = target_velocity_;
    target_current[motor] = (int) v2i_pid[motor].calc(actual_velocity_, target_velocity_);
}

void Shoot::set_friction_wheels(float duty_cycle) {
    fw_duty_cycle = duty_cycle;
}
