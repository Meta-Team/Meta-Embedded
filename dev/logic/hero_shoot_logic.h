//
// Created by 钱晨 on 2019-07-03.
//

#ifndef META_INFANTRY_HERO_SHOOT_LOGIC_H
#define META_INFANTRY_HERO_SHOOT_LOGIC_H

/**
 * @file    hero_shoot_logic.h
 * @brief   Logic-level module to control shooter.
 *
 * @addtogroup shoot
 * @{
 */

#include "ch.hpp"
#include "shell.h"
#include "shoot_scheduler.h"

class HeroShootLG {
public:

    static void init(float loader_angle_per_bullet_, float plate_angle_per_bullet_, tprio_t stuck_detector_thread_prio, tprio_t automatic_thread_prio);

    static void shoot();

    static void ForceStop();
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

    enum loader_state_t{
        LOADING,
        STOP,
        STUCK,
        FORCESTOP,
    };

    static loader_state_t loaderState;
    static loader_state_t plateState;

private:

    static int load_bullet_count;

    static float loader_angle_per_bullet;
    static float loader_target_angle;
    static float plate_angle_per_bullet;
    static float plate_target_angle;

    // Another Idea: Check whether the motor is stuck based on the motor response time.
    static int loader_runtime;
    static int plate_runtime;

    static bool loaded_bullet[4];

    class StuckDetectorThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    class AutomateThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static constexpr unsigned STUCK_DETECTOR_THREAD_INTERVAL = 10;
    static constexpr unsigned AUTOMATION_THREAD_INTERVAL = 5;
    static constexpr unsigned STUCK_REVERSE_TIME = 250;

    static constexpr int LOADER_STUCK_THRESHOLD_CURRENT = 1500;
    static constexpr int LOADER_STUCK_THRESHOLD_VELOCITY = 2;

    static constexpr int PLATE_STUCK_THRESHOLD_CURRENT = 3000;
    static constexpr int PLATE_STUCK_THRESHOLD_VELOCITY = 2;

    static StuckDetectorThread stuckDetector;
    static AutomateThread automation;

};


#endif //META_INFANTRY_HERO_SHOOT_LOGIC_H
