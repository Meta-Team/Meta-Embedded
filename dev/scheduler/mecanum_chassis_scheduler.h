//
// Rebuild by liuzikai
//

/**
 * @file    chassis_scheduler.h
 * @brief   Scheduler to control chassis to meet the target, including a thread to invoke PID calculation in period.
 *
 * @addtogroup chassis
 * @{
 */

#ifndef META_INFANTRY_CHASSIS_CONTROLLER_H
#define META_INFANTRY_CHASSIS_CONTROLLER_H

#include "can_motor_scheduler.h"

#if defined(INFANTRY)
#include "vehicle/infantry/vehicle_infantry.h"
#elif defined(HERO)
#include "vehicle_hero.h"
# elif defined(ut_chassis)

#else
#error "Files infantry_shoot_logic.h/cpp can only be used for Infantry or Hero main program now"
#endif

/**
 * @name ChassisSKD
 * @note SKD stands for "scheduler"
 * @brief Scheduler to control chassis to meet the target, including a thread to invoke PID calculation in period.
 * @pre GimbalIF and ChassisIF have been initialized properly
 * @usage 1. init()
 *        2. set_target_angle() as needed
 *        3. Leave the rest of the work to this SKD
 * @updates 1. Installation direction and PID parameter configuration left in CAN_MOTOR_CFG.
 *          2. SKD thread only handle velocity decompose [vx, vy, omega (angular velocity)], but not for chassis follow, dodge.
 *          TODO: Re-enable chassis shell functions.
 */
class MecanumChassisSKD {

public:

    static enum install_mode_t{
        POSITIVE =  1,
        NEGATIVE = -1,
    } install_mode;

    /**
     * Initialize this module
     * @param skd_prio priority for scheduler thread
     * @param wheel_base            Distance between front axle and the back axle [mm]
     * @param wheel_thread          Distance between left and right wheels [mm]
     * @param wheel_circumference   Circumference of wheels [mm]
     */
    static void init(tprio_t skd_prio, float wheel_base, float wheel_thread, float wheel_circumference);

    /**
     * Set target values
     * @param vx     Target velocity along the x axis (right) with respect to gimbal coordinate [mm/s]
     * @param vy     Target velocity along the y axis (up) with respect to gimbal coordinate [mm/s]
     * @param omega  Target angular velocity [degree/s]
     */
    static void set_target(float vx, float vy, float omega);

    static float w_to_v_ratio;

    static float v_to_wheel_angular_velocity;

/// TODO: Re-Enable These functions.
#ifdef ENABLE_CHASSIS_SHELL
    static void cmdFeedback(void *);
    static const Shell::Command shellCommands[];
#endif
private:
    static PIDController dodge_omega_power_pid;

    static float target_vx;
    static float target_vy;
    static float target_omega;

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned SKD_INTERVAL = 5; //[ms]
        void main() final;
    };

    static SKDThread skd_thread;

/// TODO: Re-Enable These functions.
#ifdef ENABLE_CHASSIS_SHELL
    static bool motor_enabled;
    static DECL_SHELL_CMD(cmdInfo);
    static DECL_SHELL_CMD(cmdEnableFeedback);
    static DECL_SHELL_CMD(cmdPID);
    static DECL_SHELL_CMD(cmdEnableMotor);
#endif
};

#endif //META_INFANTRY_CHASSIS_CONTROLLER_H

/** @} */
