//
// Created by liuzikai on 2019-06-26.
//

#ifndef META_INFANTRY_SHOOT_LOGIC_H
#define META_INFANTRY_SHOOT_LOGIC_H

#include "ch.hpp"
#include "shell.h"
#include "shoot_scheduler.h"

class ShootLG {

public:

    static void init(float angle_per_bullet_, tprio_t stuck_detector_thread_prio);

    static void increment_bullet(int number_of_bullet);

    static void set_bullet_count(int number_of_bullet);

    static int get_bullet_count();

    /**
     * Set friction wheel duty cycle in LIMITED_SHOOTING_MODE or REVERSE_TURNING_MODE
     * @param duty_cycle  Friction wheel duty cycle, from 0 to 1.0
     */
    static void set_friction_wheels(float duty_cycle);

    enum shooter_state_t {
        STOP,
        SHOOTING,
        STUCK
    };

    static shooter_state_t get_shooter_state();

    static void shoot(int number_of_bullet);

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


#endif //META_INFANTRY_SHOOT_LOGIC_H
