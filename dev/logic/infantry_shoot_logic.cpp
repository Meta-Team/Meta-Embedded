//
// Created by liuzikai on 2019-06-26.
//

/**
 * @file    shoot_logic.cpp
 * @brief   Logic-level module to control shooter. Support number-controlling shooting.
 *
 * @addtogroup shoot
 * @{
 */


#include "infantry_shoot_logic.h"

float ShootLG::angle_per_bullet = 0;
int ShootLG::bullet_count = 0;
int ShootLG::shoot_target_number = 0;
ShootLG::shooter_state_t ShootLG::shooter_state = STOP;
ShootLG::StuckDetectorThread ShootLG::stuckDetector;

void ShootLG::init(float angle_per_bullet_, tprio_t stuck_detector_thread_prio) {
    angle_per_bullet = angle_per_bullet_;
    stuckDetector.start(stuck_detector_thread_prio);
}

void ShootLG::increment_bullet(int number_of_bullet) {
    bullet_count += number_of_bullet;
}

void ShootLG::set_bullet_count(int number_of_bullet) {
    bullet_count = number_of_bullet;
}

int ShootLG::get_bullet_count() {
    return bullet_count;
}

void ShootLG::set_friction_wheels(float duty_cycle) {
    ShootSKD::set_friction_wheels(duty_cycle);
}

float ShootLG::get_friction_wheels_duty_cycle() {
    return ShootSKD::get_friction_wheels_duty_cycle();
}

ShootLG::shooter_state_t ShootLG::get_shooter_state() {
    return shooter_state;
}

void ShootLG::shoot(int number_of_bullet) {
    shoot_target_number = number_of_bullet;
    shooter_state = SHOOTING;
    ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::set_loader_target(shoot_target_number * angle_per_bullet);
}

void ShootLG::stop() {
    ShootSKD::set_mode(ShootSKD::FORCED_RELAX_MODE);
    bullet_count -= (int) (ShootSKD::get_loader_accumulated_angle() / angle_per_bullet);
    ShootSKD::reset_loader_accumulated_angle();
    shooter_state = STOP;
}

void ShootLG::StuckDetectorThread::main() {
    setName("Shoot_Stuck");
    while (!shouldTerminate()) {

        if (shooter_state == SHOOTING &&
            ShootSKD::get_loader_target_current() > STUCK_THRESHOLD_CURRENT &&
            ShootSKD::get_loader_actual_velocity() < STUCK_THRESHOLD_VELOCITY) {

            shooter_state = STUCK;
            LOG_WARN("Bullet loader stuck");
            ShootSKD::set_loader_target(ShootSKD::get_loader_accumulated_angle() - STUCK_REVERSE_ANGLE);
            sleep(TIME_MS2I(STUCK_REVERSE_TIME));
            if (shooter_state == STOP) {  // if shooter is stopped during the pausing time, terminate the process.
                continue;
            }
            ShootSKD::set_loader_target(shoot_target_number * angle_per_bullet);  // recover original target
            shooter_state = SHOOTING;

        }

        sleep(TIME_MS2I(STUCK_DETECTOR_THREAD_INTERVAL));
    }
}

/** @} */
