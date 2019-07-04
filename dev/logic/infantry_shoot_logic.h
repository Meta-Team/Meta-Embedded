//
// Created by liuzikai on 2019-06-26.
//

#ifndef META_INFANTRY_INFANTRY_SHOOT_LOGIC_H
#define META_INFANTRY_INFANTRY_SHOOT_LOGIC_H

/**
 * @file    infantry_shoot_logic.h
 * @brief   Logic-level module to control shooter. Support number-controlling shooting.
 *
 * @addtogroup shoot
 * @{
 */

#include "ch.hpp"
#include "shell.h"
#include "shoot_scheduler.h"

/**
 * @name ShootLG
 * @note LG stands for "logic"
 * @brief Logic-level module to control shooter. Support number-controlling shooting.
 * @pre ShootSKD has started properly
 * @usage 1. Invoke init()
 *        2. Invoke set_action() and set_target() to control gimbal
 */
class ShootLG {

public:

    /**
     * Initialize this module
     * @param angle_per_bullet_            Angle for bullet loader to rotate to fill one bullet [degree]
     * @param stuck_detector_thread_prio   Thread priority for stuck detector thread
     */
    static void init(float angle_per_bullet_, tprio_t stuck_detector_thread_prio);

    /**
     * Add bullet to internal bullet counter
     * @param number_of_bullet   Number of bullets to added
     */
    static void increment_bullet(int number_of_bullet);

    /**
     * Set value of internal bullet counter
     * @param number_of_bullet   Value of bullets
     */
    static void set_bullet_count(int number_of_bullet);

    /**
     * Return the value of internal bullet counter
     * @return Number of remaining bullet
     */
    static int get_bullet_count();

    /**
     * Set friction wheels duty cycle in LIMITED_SHOOTING_MODE or REVERSE_TURNING_MODE
     * @param duty_cycle  Friction wheels duty cycle, from 0 to 1.0
     */
    static void set_friction_wheels(float duty_cycle);

    /**
     * Get friction wheels duty cycle
     * @return Friction wheels duty cycle, from 0 to 1.0
     */
    static float get_friction_wheels_duty_cycle();

    enum shooter_state_t {
        STOP,
        SHOOTING,
        STUCK
    };

    /**
     * Get current shooter state
     * @return Current shooter state
     */
    static shooter_state_t get_shooter_state();

    /**
     * Shoot bullets. Will change shooter state.
     * @param number_of_bullet   Number of bullet to shoot.
     */
    static void shoot(int number_of_bullet);

    /**
     * Stop shooting. Will change shooter state.
     */
    static void stop();

private:

    static float angle_per_bullet;

    static int bullet_count;
    static int shoot_target_number;

    static shooter_state_t shooter_state;


    static constexpr unsigned STUCK_DETECTOR_THREAD_INTERVAL = 10;  // [ms]

    static constexpr int STUCK_THRESHOLD_CURRENT = 1500;  // lower current to trigger stuck handling [mA]
    static constexpr float STUCK_THRESHOLD_VELOCITY = 5;  // upper velocity to trigger stuck handling [degree/s]

    static constexpr unsigned STUCK_REVERSE_TIME = 500;  // time to stay in reverse turing state [ms]
    static constexpr unsigned STUCK_REVERSE_ANGLE = 5;   // reverse turning target angle when stuck [degree]

    class StuckDetectorThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static StuckDetectorThread stuckDetector;

};

#endif //META_INFANTRY_INFANTRY_SHOOT_LOGIC_H

/** @} */