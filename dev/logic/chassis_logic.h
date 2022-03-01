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
#include <cmath>

#if(FALSE)
#include "referee_interface.h"
#include "referee_UI_logic.h"
#include "super_capacitor_port.h"
#endif
#include "pid_controller.hpp"
#include "mecanum_chassis_scheduler.h"

#if defined(INFANTRY)
#include "vehicle/infantry/vehicle_infantry.h"
#elif defined(HERO)
#include "vehicle_hero.h"
# elif defined(ut_chassis)

#else
#error "Files infantry_shoot_logic.h/cpp can only be used for Infantry or Hero main program now"
#endif

/**
 * @name ChassisLG
 * @note LG stands for "logic"
 * @brief Logic-level module to generate target values for ChassisSKD. Support follow-gimbal mode and dodge mode.
 * @pre ChassisSKD has started properly
 * @usage 1. Invoke init()
 *        2. Invoke set_action() and set_target_angle() to control chassis
 */
class ChassisLG {

public:

    enum chassis_mode_t {
        FORCE_RELAX,
        ABS,
        FOLLOW,
        DODGE
    };

    /**
     * Initialize this module
     * @param dodge_thread_prio_   Thread priority for dodge thread
     * @param cap_power_set_thread_prio_ Thread priority for the capacitor power set thread
     * @param dodge_mode_max_omega     Max Rotation angular speed (omega) in DODGE_MODE [degree/s]
     */
    static void init(tprio_t dodge_thread_prio_, tprio_t cap_power_set_thread_prio_, tprio_t dodge_mode_max_omega);

    static void set_mode(chassis_mode_t mode);

    /**
     * Set the target velocity in gimbal coordinate
     * @param vx vx in gimbal coordinate
     * @param vy vy in gimbal coordinate
     */
    static void set_target(float vx, float vy);

    static void set_target_omega(float omega);
#if(FALSE)
    static void init(tprio_t dodge_thread_prio_, tprio_t cap_power_set_thread_prio_, float dodge_mode_max_omega, float biased_angle, PIDController::pid_params_t omega_power_pid);

    enum action_t {
        FORCED_RELAX_MODE,
        FOLLOW_MODE,
        DODGE_MODE,
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
#endif

private:
    static PIDController dodge_omega_power_pid;

    static float target_vx;
    static float target_vy;
    static float target_omega;

    static chassis_mode_t chassis_mode;

    class ChassisLGThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned VEL_DECOMPOSE_INTERVAL = 5; //[ms]
        void main() final;
    };

    // TODO Re-enable the functions.
#if (FALSE)
    static float target_theta;
    static action_t action;
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

        static constexpr unsigned DODGE_MODE_SWITCH_INTERVAL = 5;  // interval to switch direction in DODGE_MODE [ms]

        void main() final;
    };

    class CapacitorPowerSetThread : public chibios_rt::BaseStaticThread<512> {
    public:
    private:
        static constexpr unsigned CAP_POWER_SET_INTERVAL = 100; //[ms]
        void main() final;
    };

    static CapacitorPowerSetThread capacitorPowerSetThread;
    static DodgeModeSwitchThread dodgeModeSwitchThread;
    static chibios_rt::ThreadReference dodgeThreadReference;
#endif
    static ChassisLGThread chassis_logic_thread;
};


#endif //META_INFANTRY_CHASSIS_LOGIC_H

/** @} */