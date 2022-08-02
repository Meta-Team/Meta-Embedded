//
// Rebuild by liuzikai
// Rebuild by Chen Qian on Mar. 16, 2022
//

/**
 * @file    chassis_scheduler.h
 * @brief   Scheduler to control chassis to meet the target, including a thread to invoke PID calculation in period.
 *
 * @version 3.0a
 * @date 03.16, 2022
 * @addtogroup chassis
 * @{
 */

#ifndef META_INFANTRY_CHASSIS_CONTROLLER_H
#define META_INFANTRY_CHASSIS_CONTROLLER_H

#include "can_motor_scheduler.h"
#include "scheduler_base.h"
#include <cmath>

#if defined(INFANTRY)
#include "vehicle_infantry.h"
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
 * @date 3.16, 2022
 * @version 3.0a
 * @usage 1. init()
 *        2. set_target_angle() as needed
 *        3. Leave the rest of the work to this SKD
 * @updates 1. Installation direction and PID parameter configuration left in CAN_MOTOR_CFG.
 *          2. SKD thread only handle velocity decompose [vx, vy, omega (angular velocity)], but not for chassis follow, dodge.
 *          TODO: Re-enable chassis shell functions.
 */
class MecanumChassisSKD : public SKDBase{

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
    static void init(tprio_t skd_prio, float wheel_base, float wheel_thread, float wheel_circumference, float gimbal_offset = 0);

    /**
     * Set the mode for chassis.
     * @param mode_
     */
    static void set_mode(mode_t mode_);

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

    /**
     * @brief Input target velocity in X axis (orthogonal to gimbal vehicle's direction)
     */
    static float target_vx;

    /**
     * @brief Input target velocity in X axis (parallel to gimbal vehicle's direction)
     */
    static float target_vy;

    /**
     * @brief Input target chassis angular velocity.
     */
    static float target_omega;

    /**
     * @brief The distance of chassis and gimbal's geometric centers.
     */
    static float chassis_gimbal_offset_;

    /**
     * @brief Thread for performing velocity decompose and angle transformation.
     */
    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned SKD_INTERVAL = 5; //[ms]
        void main() final;
    };

    static SKDThread skd_thread;

    /// Gimbal Behavior Control
    static mode_t mode;

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
