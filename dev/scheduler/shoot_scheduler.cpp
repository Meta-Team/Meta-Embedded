//
// Created by liuzikai on 2019-05-01.
//

#include "shoot_scheduler.h"

ShootSKD::install_direction_t ShootSKD::install_position[2];

ShootSKD::mode_t ShootSKD::mode = FORCED_RELAX_MODE;

float ShootSKD::target_angle[2] = {0, 0};
float ShootSKD::target_velocity[2] = {0, 0};
int ShootSKD::target_current[2] = {0, 0};
float ShootSKD::target_fw = 0;

PIDController ShootSKD::v2i_pid[2];
PIDController ShootSKD::a2v_pid[2];

ShootSKD::SKDThread ShootSKD::skdThread;

void ShootSKD::start(ShootSKD::install_direction_t loader_install_, ShootSKD::install_direction_t plate_install_,
                     tprio_t thread_prio) {
    install_position[0] = loader_install_;
    install_position[1] = plate_install_;

    skdThread.start(thread_prio);
}

void ShootSKD::load_pid_params(pid_params_t loader_a2v_params,
                               pid_params_t loader_v2i_params,
                               pid_params_t plate_a2v_params,
                               pid_params_t plate_v2i_params) {
    v2i_pid[0].change_parameters(loader_v2i_params);
    v2i_pid[1].change_parameters(plate_v2i_params);
    a2v_pid[0].change_parameters(loader_a2v_params);
    a2v_pid[1].change_parameters(plate_a2v_params);
}

void ShootSKD::set_mode(ShootSKD::mode_t skd_mode) {
    mode = skd_mode;
}

ShootSKD::mode_t ShootSKD::get_mode() {
    return mode;
}

void ShootSKD::set_loader_target(float loader_target_angle) {
    target_angle[0] = loader_target_angle;
}

void ShootSKD::set_plate_target(float plate_target_angle) {
    target_angle[1] = plate_target_angle;
}

void ShootSKD::set_friction_wheels(float duty_cycle) {
    target_fw = duty_cycle;
}

int ShootSKD::get_loader_target_current() {
    return target_current[0];
}

int ShootSKD::get_plate_target_current() {
    return target_current[1];
}

float ShootSKD::get_loader_actual_velocity() {
    return GimbalIF::feedback[GimbalIF::BULLET].actual_velocity * install_position[0];
}

float ShootSKD::get_plate_actual_velocity() {
    return GimbalIF::feedback[GimbalIF::PLATE].actual_velocity * install_position[1];
}

float ShootSKD::get_loader_accumulated_angle() {
    return GimbalIF::feedback[GimbalIF::BULLET].accumulate_angle() * install_position[0];
}

float ShootSKD::get_plate_accumulated_angle() {
    return GimbalIF::feedback[GimbalIF::PLATE].accumulate_angle() * install_position[1];
}

void ShootSKD::reset_loader_accumulated_angle() {
    GimbalIF::feedback[GimbalIF::BULLET].reset_front_angle();
}

void ShootSKD::reset_plate_accumulated_angle() {
    GimbalIF::feedback[GimbalIF::PLATE].reset_front_angle();
}

void ShootSKD::SKDThread::main() {
    setName("shoot_skd");
    while (!shouldTerminate()) {

        if (mode == LIMITED_SHOOTING_MODE) {

            // PID calculation
            for (size_t i = 0; i < 2; i++) {
                target_velocity[i] = a2v_pid->calc(GimbalIF::feedback[2 + i].accumulate_angle() * install_position[i],
                                                   target_angle[i]);
                target_current[i] = (int) v2i_pid->calc(
                        GimbalIF::feedback[2 + i].actual_velocity * install_position[i],
                        target_velocity[i]);
                GimbalIF::target_current[i + 2] = target_current[i] * install_position[i];
            }

            GimbalIF::fw_duty_cycle = target_fw;

        } else if (mode == FORCED_RELAX_MODE) {

            GimbalIF::target_current[GimbalIF::BULLET] = GimbalIF::target_current[GimbalIF::PLATE] = 0;
            GimbalIF::fw_duty_cycle = 0;

        }

        // Send currents with GimbalSKD (has smaller SKD_THREAD_INTERVAL)

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}

