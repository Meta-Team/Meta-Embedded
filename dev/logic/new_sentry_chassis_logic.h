//
// Created by kerui on 2021/7/29.
//

#ifndef META_INFANTRY_NEW_SENTRY_CHASSIS_LOGIC_H
#define META_INFANTRY_NEW_SENTRY_CHASSIS_LOGIC_H

#include "ch.hpp"
#include "common_macro.h"

class SChassisLG {

public:

    static void init(tprio_t direction_switch_thd_prio);

    enum mode_t {
        FORCED_RELAX_MODE,
        MANUAL_MODE,
        SHUTTLE_MODE
    };

    static void set_mode(mode_t mode);

    static mode_t get_mode() { return mode_; };

    static void set_dest(float dest);

    static float get_dest();

private:

    static mode_t mode_;

    static float target_dest;

    class DirectionSwitchThread : public chibios_rt::BaseStaticThread<512> {
    public:

        bool started = false;

    private:

        // interval to evaluate whether switching dest is needed in SHUTTLE_MODE or FINAL_AUTO_MODE [ms]
        static constexpr unsigned DIRECTION_INSPECTION_INTERVAL = 5;

        void main() final;
    };

    static DirectionSwitchThread directionSwitchThread;
};


#endif //META_INFANTRY_NEW_SENTRY_CHASSIS_LOGIC_H
