//
// Created by liuzikai on 2019-05-01.
// Mo Kanya revised the friction wheel part.
//

/**
 * @file    shoot_scheduler.cpp
 * @brief   Scheduler to control shooter to meet the target, including a thread to invoke PID calculation in period.
 *
 * @addtogroup shoot
 * @{
 */

#include "shoot_scheduler.h"

ShootSKD::install_direction_t ShootSKD::install_position[3];

ShootSKD::mode_t ShootSKD::mode = FORCED_RELAX_MODE;
bool ShootSKD::is_test = false;
bool ShootSKD::motor_enable[3] = {false, false, false};

float ShootSKD::target_angle = 0;
float ShootSKD::target_velocity[3] = {0, 0, 0};
int ShootSKD::target_current[3] = {0, 0, 0};

PIDController ShootSKD::v2i_pid[3];
PIDController ShootSKD::a2v_pid;

ShootSKD::SKDThread ShootSKD::skd_thread;

void ShootSKD::start(ShootSKD::install_direction_t loader_install_, tprio_t thread_prio) {
    install_position[0] = loader_install_;
    install_position[1] = NEGATIVE;
    install_position[2] = POSITIVE;

    skd_thread.start(thread_prio);
}

void ShootSKD::load_pid_params(pid_params_t loader_a2v_params,
                               pid_params_t loader_v2i_params,
                               pid_params_t fw_left_v2i_params,
                               pid_params_t fw_right_v2i_params) {

    v2i_pid[0].change_parameters(loader_v2i_params);
    a2v_pid.change_parameters(loader_a2v_params);
    v2i_pid[1].change_parameters(fw_left_v2i_params);
    v2i_pid[2].change_parameters(fw_right_v2i_params);
}

void ShootSKD::load_pid_params_by_type(pid_params_t params, unsigned int motor_id, bool is_a2v) {
    if (is_a2v) {
        if (motor_id == 0)
            a2v_pid.change_parameters(params);
    } else {
        v2i_pid[motor_id].change_parameters(params);
    }
}

PIDController::pid_params_t ShootSKD::echo_pid_params_by_type(unsigned motor_id, bool is_a2v) {
    if (is_a2v) {
        if (motor_id == 0)
            return a2v_pid.get_parameters();
        else
            return {0,0,0,0,0};
    } else {
        return v2i_pid[motor_id].get_parameters();
    }
}

void ShootSKD::set_mode(ShootSKD::mode_t skd_mode) {
    mode = skd_mode;
}

ShootSKD::mode_t ShootSKD::get_mode() {
    return mode;
}

void ShootSKD::set_loader_target_angle(float loader_target_angle) {
    target_angle = loader_target_angle;
}

void ShootSKD::set_loader_target_velocity(float degree_per_second) {
    pid_params_t p = a2v_pid.get_parameters();
    p.out_limit = degree_per_second;
    a2v_pid.change_parameters(p);
    a2v_pid.clear_i_out();
}

void ShootSKD::set_friction_wheels(float duty_cycle) {
    target_velocity[1] = -duty_cycle *3000.0f;
    target_velocity[2] = target_velocity[1];
}

float ShootSKD::get_friction_wheels_duty_cycle() {
    return target_velocity[2] / 3000.0f;
}

float ShootSKD::get_target_velocity(uint32_t motor_id) {
    return target_velocity[motor_id];
}

int ShootSKD::get_target_current(uint32_t motor_id) {
    return ShootSKD::target_current[motor_id];
}

float ShootSKD::get_actual_velocity(uint32_t motor_id) {
    return GimbalIF::feedback[motor_id + 3]->actual_velocity * float(install_position[motor_id]);
}

float ShootSKD::get_loader_target_angle() {
    return target_angle;
}

float ShootSKD::get_loader_accumulated_angle() {
    return GimbalIF::feedback[GimbalIF::BULLET]->accumulated_angle() * float(install_position[0]);
}

void ShootSKD::reset_loader_accumulated_angle() {
    GimbalIF::feedback[GimbalIF::BULLET]->reset_front_angle();
}

void ShootSKD::set_test_status(bool test_status) {
    is_test = test_status;
}

void ShootSKD::enable_motor(unsigned motor) {
    motor_enable[motor] = true;
}

void ShootSKD::disable_motor(unsigned motor) {
    motor_enable[motor] = false;
}

void ShootSKD::SKDThread::main() {
    setName("ShootSKD");
    while (!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            if (mode == LIMITED_SHOOTING_MODE) {

                // PID calculation

                // Bullet calculation
                if (!is_test || motor_enable[0]) {
                    target_velocity[0] = a2v_pid.calc(
                            GimbalIF::feedback[GimbalIF::BULLET]->accumulated_angle() * float(install_position[0]),
                            target_angle);
                    target_current[0] = (int) v2i_pid[0].calc(
                            GimbalIF::feedback[GimbalIF::BULLET]->actual_velocity * float(install_position[0]),
                            target_velocity[0]);
                }
                // Fraction wheels calculation
                if (!is_test || motor_enable[1]) {
                    ShootSKD::target_current[1] = (int) v2i_pid[1].calc(
                            GimbalIF::feedback[GimbalIF::FW_LEFT]->actual_velocity,
                            target_velocity[1] * float(install_position[1]));
                }
                if (!is_test || motor_enable[2]) {
                    ShootSKD::target_current[2] = (int) v2i_pid[2].calc(
                            GimbalIF::feedback[GimbalIF::FW_RIGHT]->actual_velocity,
                            target_velocity[2] * float(install_position[2]));
                }

            } else if (mode == FORCED_RELAX_MODE) {
                target_current[0] = 0;

                target_current[1] = ABS_IN_RANGE(GimbalIF::feedback[GimbalBase::FW_LEFT]->actual_velocity, 100) ?
                        0 : (int) v2i_pid[1].calc(GimbalIF::feedback[GimbalBase::FW_LEFT]->actual_velocity,0.0f);

                target_current[2] = ABS_IN_RANGE(GimbalIF::feedback[GimbalBase::FW_RIGHT]->actual_velocity, 100) ?
                                    0 : (int) v2i_pid[2].calc(GimbalIF::feedback[GimbalBase::FW_RIGHT]->actual_velocity,0.0f);
            }

            if (is_test) {
                for (int i = 0; i <= 2; i++) {
                    target_current[i] = motor_enable[i] ? target_current[i] : 0;
                }
            }

            *GimbalIF::target_current[GimbalIF::BULLET] = target_current[0] * install_position[0];
            *GimbalIF::target_current[GimbalIF::FW_LEFT] = ShootSKD::target_current[1];
            *GimbalIF::target_current[GimbalIF::FW_RIGHT] = ShootSKD::target_current[2];
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}

/*void ShootSKD::set_mag_cover(float angle, GimbalIF::MG995_loc_t MG995) {
    float duty_cycle = angle/180;
    GimbalIF::setPWM(duty_cycle, MG995);
}*/

/** @} */