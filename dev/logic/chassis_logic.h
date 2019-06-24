//
// Created by liuzikai on 2019-06-15.
//

/**
 * @file    chassis_logic.h
 * @brief   Generate target values for ChassisSKD.
 *
 * @addtogroup chassis
 * @{
 */

#ifndef META_INFANTRY_CHASSIS_LOGIC_H
#define META_INFANTRY_CHASSIS_LOGIC_H

#include "ch.hpp"
#include "chassis_scheduler.h"

class ChassisLG {

public:

    static void init(tprio_t dodge_thread_prio_);

    enum action_t {
        FOLLOW_MODE,
        DODGE_MODE
    };

    static action_t get_action();

    static void set_action(action_t value);

    /**
     * Set target values
     * @param vx     Target velocity along the x axis (right) with respect to gimbal coordinate [mm/s]
     * @param vy     Target velocity along the y axis (up) with respect to gimbal coordinate [mm/s]
     */
    static void set_target(float vx, float vy);

private:

    static action_t action;
    static float target_vx;
    static float target_vy;
    static float target_theta;

    static void apply_target();  // helper function to apply target values to ChassisSKD

    static constexpr unsigned DODGE_MODE_THETA = 45;  // rotation angle (theta) in DODGE_MODE [degree]

    static tprio_t dodge_thread_prio;

    class DodgeModeSwitchThread : public chibios_rt::BaseStaticThread<512> {
    public:

        bool started = false;

    private:

        static constexpr unsigned DODGE_MODE_SWITCH_INTERVAL = 1000;  // interval to switch direction in DODGE_MODE [ms]

        void main() final;
    };

    static DodgeModeSwitchThread dodgeModeSwitchThread;

};


#endif //META_INFANTRY_CHASSIS_LOGIC_H

/** @} */