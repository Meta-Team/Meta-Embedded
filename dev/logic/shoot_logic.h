//
// Created by liuzikai on 2019-06-26.
//

#ifndef META_INFANTRY_SHOOT_LOGIC_H
#define META_INFANTRY_SHOOT_LOGIC_H

/**
 * @file    infantry_shoot_logic.h
 * @brief   Logic-level module to control shooter. Support number-controlling shooting.
 *
 * @addtogroup shoot
 * @{
 */

#include "ch.hpp"

#if defined(INFANTRY)
#include "vehicle_infantry.h"
#elif defined(SENTRY)
#include "vehicle_sentry.h"
#elif defined(AERIAL)
#include "vehicle_aerial.h"
#elif defined(HERO)
#include "vehicle_hero.h"
#else
#error "Files infantry_shoot_logic.h/cpp should only be used for Infantry or Sentry main program"
#endif

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

    enum limit_mode_t {
        UNLIMITED_MODE,      // the number of bullets and the feed rate is set by shoot()
        VISION_LIMITED_MODE  // shoot up to the target set up shoot() if allowed by Vision
    };

    static limit_mode_t get_limit_mode() { return mode; }

    static void set_limit_mode(limit_mode_t new_mode) { mode = new_mode; }

    /**
     * Initialize this module
     * @param angle_per_bullet_            Angle for bullet loader to rotate to fill one bullet [degree]
     * @param use_42mm_bullet
     * @param stuck_detector_thread_prio   Thread priority for stuck detector thread
     * @param bullet_counter_thread_prio   Thread priority for bullet counter thread
     * @param vision_shooting_thread_prio  Thread priority for Vision automatic shooting thread
     */
    static void init(float angle_per_bullet_, bool use_42mm_bullet, tprio_t stuck_detector_thread_prio,
                     tprio_t bullet_counter_thread_prio, tprio_t vision_shooting_thread_prio);

    /**
     * Add bullet to internal bullet counter
     * @param number_of_bullet   Number of bullets to added
     */
    static void increment_remaining_bullet(int number_of_bullet);

    /**
     * Set value of internal bullet counter
     * @param number_of_bullet   Value of bullets
     */
    static void set_remaining_bullet_count(int number_of_bullet);

    /**
     * Return the value of internal bullet counter
     * @return Number of remaining bullet
     */
    static int get_remaining_bullet_count();

    /**
     * Set friction wheels duty cycle in LIMITED_SHOOTING_MODE or REVERSE_TURNING_MODE
     * @param speed  [deg/s]
     */
    static void set_shoot_speed(float speed);

    /**
     * Get friction wheels duty cycle
     * @return Shoot speed [deg/s]
     */
    static float get_shoot_speed();

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
    static void shoot(float number_of_bullet, float number_per_second);

    /**
     * Stop shooting.
     */
    static void stop();

    /**
     * Stop shooting immediately. Will change shooter state.
     */
    static void force_stop();

    static int get_bullet_count_to_heat_limit();

    /**
     * open mag cover
     */
    static void mag_open();

    /**
     * close mag cover
     */
    static void mag_close();

private:

    static limit_mode_t mode;
    static float angle_per_bullet;
    static int remaining_bullet_count;
    static float target_bullet_count;
    static float target_bullet_loader_velocity;
    static shooter_state_t shooter_state;

    static bool use_42mm_bullet;
    static constexpr int HEAT_PER_17MM_BULLET = 10;
    static constexpr int HEAT_PER_42MM_BULLET = 100;


    /// Stuck Detector
    class StuckDetectorThread : public chibios_rt::BaseStaticThread<512> {
    public:

        bool started = false;
        bool paused_once = false;

    private:
        long int stuck_count = 0;

        static constexpr unsigned STUCK_DETECTOR_THREAD_INTERVAL = 10;  // [ms]

        static constexpr unsigned STUCK_DETECTOR_INITIAL_WAIT_INTERVAL = 500;  // [ms]

        static constexpr int STUCK_THRESHOLD_CURRENT = 7000;  // lower current to trigger stuck handling [mA]
        static constexpr float STUCK_THRESHOLD_VELOCITY = 15;  // upper velocity to trigger stuck handling [degree/s]
        static constexpr int STUCK_THRESHOLD_COUNT = 100;

        static constexpr unsigned STUCK_REVERSE_TIME = 1000;  // time to stay in reverse turing state [ms]
        static constexpr unsigned STUCK_REVERSE_ANGLE = 15;   // reverse turning target angle when stuck [degree]

        void main() final;
    };

    static StuckDetectorThread stuck_detector_thread;
    static chibios_rt::ThreadReference stuck_detector_ref;


    /// Bullet Counter using Referee Data
    class BulletCounterThread : public chibios_rt::BaseStaticThread<256> {

        event_listener_t data_received_listener;
        static constexpr eventmask_t DATA_RECEIVED_EVENTMASK = EVENT_MASK(0);

        void main() final;
    };

    static BulletCounterThread bullet_counter_thread;

    /// Vision-Controlled Shooting
    class VisionShootThread : public chibios_rt::BaseStaticThread<256> {
        event_listener_t vision_listener;
        static constexpr time_msecs_t WAIT_TIME_BETWEEN_SHOOTS = 200;   // [ms]
        void main() final;
    };

    static VisionShootThread vision_shoot_thread;

};

#endif //META_INFANTRY_SHOOT_LOGIC_H

/** @} */