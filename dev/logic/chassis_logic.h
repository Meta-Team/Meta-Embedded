//
// Created by liuzikai on 2019-06-15.
//

/**
 * @file    chassis_logic.h
 * @brief   Generate target values for ChassisSKD. Support follow-gimbal mode and dodge mode.
 *
 * @addtogroup chassis
 * @{
 */

#ifndef META_INFANTRY_CHASSIS_LOGIC_H
#define META_INFANTRY_CHASSIS_LOGIC_H

#include "ch.hpp"

#include "referee_interface.h"

#if defined(INFANTRY)
#include "vehicle_infantry.h"
#elif defined(HERO)
#include "vehicle_hero.h"
#else
#error "Files infantry_shoot_logic.h/cpp can only be used for Infantry or Hero main program now"
#endif

/**
 * @name ChassisLG
 * @note LG stands for "logic"
 * @brief Logic-level module to generate target values for ChassisSKD. Support follow-gimbal mode and dodge mode.
 * @pre ChassisSKD has started properly
 * @usage 1. Invoke init()
 *        2. Invoke set_action() and set_target() to control chassis
 */
class ChassisLG {

public:

    /**
     * Initialize this module
     * @param dodge_thread_prio_   Thread priority for dodge thread
     * @param dodge_mode_max_omega     Max Rotation angular speed (omega) in DODGE_MODE [degree/s]
     */
    static void init(tprio_t dodge_thread_prio_, float dodge_mode_max_omega, float biased_angle);

    enum action_t {
        FORCED_RELAX_MODE,
        FOLLOW_MODE,
        DODGE_MODE
    };

    /**
     * Get current action of chassis
     * @return   Current action
     */
    static action_t get_action();

    /**
     * Set action of chassis
     * @param value   Action to be applied
     */
    static void set_action(action_t value);

    /**
     * Set target values
     * @param vx     Target velocity along the x axis (right) with respect to gimbal coordinate [mm/s]
     * @param vy     Target velocity along the y axis (up) with respect to gimbal coordinate [mm/s]
     */
    static void set_target(float vx, float vy);

    static void set_sports_mode();

    static bool get_sports_mode();

private:

    static action_t action;
    static float target_vx;
    static float target_vy;
    static float target_theta;
    static float target_omega;

    static void apply_target();  // helper function to apply target values to ChassisSKD
    static float dodge_mode_max_omega_;  // rotation speed (omega) in DODGE_MODE [degree/s]
    static float dodge_mode_min_omega_;
    static int dodge_mode_randomize_max_time_;  // [ms]
    static int dodge_mode_randomize_min_time_;  // [ms]
    static float biased_angle_;    // gimbal angle in dodge mode set up for Hero

    static tprio_t dodge_thread_prio;

    class DodgeModeSwitchThread : public chibios_rt::BaseStaticThread<512> {
    public:

        bool started = false;

    private:

        static constexpr unsigned DODGE_MODE_SWITCH_INTERVAL = CHASSIS_DODGE_MODE_INTERVAL;  // interval to switch direction in DODGE_MODE [ms]

        void main() final;
    };

    static DodgeModeSwitchThread dodgeModeSwitchThread;
    static chibios_rt::ThreadReference dodgeThreadReference;
};


#endif //META_INFANTRY_CHASSIS_LOGIC_H

/** @} */