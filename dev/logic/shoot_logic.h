//
// Created by liuzikai on 2019-06-26.
//

#ifndef META_INFANTRY_SHOOT_LOGIC_H
#define META_INFANTRY_SHOOT_LOGIC_H

#include "ch.hpp"
#include "shoot_scheduler.h"

class ShootLG {

public:

    static void init(float angle_per_bullet_, tprio_t stuck_detector_thread_prio);

    static void increment_bullet(int number_of_bullet);

    static int get_bullet_count();



    /**
     * Set friction wheel duty cycle
     * @param duty_cycle from 0 to 1
     */
    static void set_friction_wheels(float duty_cycle);

private:

    static float angle_per_bullet;

    static int bullet_count;


    static constexpr unsigned STUCK_DETECTOR_THREAD_INTERVAL = 10;  // [ms]

    class StuckDetectorThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static StuckDetectorThread stuckDetector;

};


#endif //META_INFANTRY_SHOOT_LOGIC_H
