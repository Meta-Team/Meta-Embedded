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

#include "capacitor_interface.h"
#include "referee_interface.h"

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
 * @brief Logic-level module to generate target values for ChassisSKD. Support chassis-ref mode, follow-gimbal mode and dodge mode.
 * @pre ChassisSKD has started properly
 * @usage
 * @code
 * 1. Invoke init().
 * 2. Invoke set_auto_straightening_pid_params() and set_dodge_omega_power_pid() for gimbal reference mode.
 * 3. Call set_mode() and set_target()
 * @endcode
 */
class ChassisLG : PIDControllerBase{

public:

    enum chassis_mode_t {
        FORCE_RELAX_MODE,
        CHASSIS_REF_MODE,
        GIMBAL_REF_MODE,
        DODGE
    };

    /**
     * Initialize this module
     * @param dodge_thread_prio_   Thread priority for dodge thread
     * @param cap_power_set_thread_prio_ Thread priority for the capacitor power set thread
     * @param dodge_mode_max_omega     Max Rotation angular speed (omega) in DODGE_MODE [degree/s]
     */
    static void init(tprio_t dodge_thread_prio_, tprio_t cap_power_set_thread_prio_, tprio_t dodge_mode_max_omega);

    /**
     * Set the mode for chassis logic, include force relax mode, chassis reference mode, gimbal reference mode and dodge mode.
     * @param mode The mode of chassis logic.
     */
    static void set_mode(chassis_mode_t mode);

    /**
     * Set the target velocity in gimbal coordinate
     * @param vx vx in gimbal coordinate
     * @param vy vy in gimbal coordinate
     */
    static void set_target(float vx, float vy);

    /**
     * Set the PID parameters for chassis auto straightening.
     * @param params PID parameters for chassis auto straightening PID controller
     */
    static void set_auto_straightening_pid_params(pid_params_t params);

    /**
     * Set the PID parameters for chassis dodge mode omega calculation.
     * @param params PID params for chassis dodge omega calculation.
     */
    static void set_dodge_omega_power_pid(pid_params_t params);

    /**
     * Set the target angular velocity of chassis
     * @param omega target velocity for chassis.
     * @note Only available for chassis ref mode.
     */
    static void set_target_omega(float omega);

    /**
     * Get the current chassis mode.
     * @return (FORCE_RELAX/GIMBAL_REF/CHASSIS_REF/DODGE) Current chassis mode.
     */
    static chassis_mode_t get_mode();

private:
    static PIDController dodge_omega_power_pid;
    static PIDController auto_straightening_pid;

    static float target_vx;
    static float target_vy;
    static float target_omega;

    static chassis_mode_t chassis_mode;

    class MotionControllerThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned CHASSIS_LG_INTERVAL = 5; //[ms]
        void main() final;
    };
    static MotionControllerThread motion_controller_thread;

#ifdef ENABLE_REFEREE
    class CapacitorSetThread : public chibios_rt::BaseStaticThread<256> {
        static constexpr unsigned CAPACITOR_SET_INTERVAL = 200; // [ms]
        void main() final;
    };
    static CapacitorSetThread capacitor_set_thread;
#endif

};


#endif //META_INFANTRY_CHASSIS_LOGIC_H

/** @} */