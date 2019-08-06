//
// Created by 钱晨 on 2019-07-03.
//

#ifndef META_INFANTRY_HERO_SHOOT_LOGIC_H
#define META_INFANTRY_HERO_SHOOT_LOGIC_H

/**
 * @file    hero_shoot_logic.h
 * @brief   Logic-level module to control shooter and realize auto bullet loading.
 * @note    Bullet plate is am additional organ in Hero's loading system.
 * @note    3 bullets can be stably held in Hero's barrel.

 * @addtogroup shoot
 * @{
 */

#include "ch.hpp"

#if defined(HERO)
#include "vehicle_hero.h"
#else
#error "Files hero_shoot_logic.h/cpp should only be used for Hero main program"
#endif

class HeroShootLG {
public:

    /**
     * Initialize this module
     * @param loader_angle_per_bullet_     Angle for bullet loader to rotate to fill one bullet [degree]
     * @param plate_angle_per_bullet_      Angle for bullet plate to rotate to fill one bullet [degree]
     * @param loader_thread_prio           Thread priority for loader thread
     * @param plate_thread_prio            Thread priority for plate thread
     * @param stuck_detector_thread_prio   Thread priority for stuck detector thread
     */
    static void init(float loader_angle_per_bullet_, float plate_angle_per_bullet_, tprio_t loader_calibrate_prio,
            tprio_t loader_thread_prio, tprio_t plate_thread_prio,
            tprio_t loader_stuck_detector_prio, tprio_t plate_stuck_detector_prio);


    static void shoot();

    /**
     * Clear bullet count and target angle and reset accumulated angle
     */
    static void force_stop();

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

    enum motor_state_t{
        LOADING,
        STOP,
        STUCK,
    };
    /**
     * Get loading status
     * @return loaded complete, true; loading/ not loaded, false.
     */
    static bool get_loading_status();

    static motor_state_t loader_state;
    static motor_state_t plate_state;

private:

    static bool get_loader_exit_status();
    static bool get_plate_exit_status();

    static float measure_loader_exit_status();

    static float loader_angle_per_bullet;
    static float loader_target_angle;
    static float plate_angle_per_bullet;
    static float plate_target_angle;

    static int bullet_in_tube;
    static bool should_shoot;

    /// Loader Calibrate Thread.

    class LoaderCalibrateThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr int CALIBRATE_THREAD_INTERVAL = 5;
        void main() final;
    };

    static LoaderCalibrateThread loaderCalibrateThread;

    /// Loader Stuck Detector
    class LoaderStuckDetectorThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned STUCK_DETECTOR_THREAD_INTERVAL = 10;

        static constexpr unsigned STUCK_REVERSE_TIME = 1000;
        static constexpr unsigned STUCK_REVERSE_ANGLE = 15;   // reverse turning target angle when stuck [degree]

        static constexpr int LOADER_STUCK_THRESHOLD_CURRENT = 2500;
        static constexpr int LOADER_STUCK_THRESHOLD_VELOCITY = 2;

        void main() final;
    };
    static LoaderStuckDetectorThread loaderStuckDetector;

    /// Plate Stuck Detector
    class PlateStuckDetectorThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned STUCK_DETECTOR_THREAD_INTERVAL = 10;

        static constexpr unsigned STUCK_REVERSE_TIME = 1000;
        static constexpr unsigned STUCK_REVERSE_ANGLE = 15;   // reverse turning target angle when stuck [degree]

        static constexpr int PLATE_STUCK_THRESHOLD_CURRENT = 2500;
        static constexpr int PLATE_STUCK_THRESHOLD_VELOCITY = 2;

        void main() final;
    };
    static PlateStuckDetectorThread plateStuckDetector;

    /// Loader Thread
    class LoaderThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned LOADER_THREAD_INTERVAL = 5;  // [ms]
        static constexpr float LOADER_REACH_TARGET_RANGE = 3.5f;  // [degree]
        void main() final;
    };
    static LoaderThread loaderThread;

    /// Plate Thread
    class PlateThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned PLATE_THREAD_INTERVAL = 5;  // [ms]
        static constexpr float PLATE_REACH_TARGET_RANGE = 3.0f;  // [degree]
        bool last_plate_exit_status = false;
        void main() final;
    };
    static PlateThread plateThread;

};


#endif //META_INFANTRY_HERO_SHOOT_LOGIC_H

/** @} */