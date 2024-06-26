//
// Created by liuzikai on 2019-05-01.
// Mo Kanya revised the friction wheel part.
//

/**
 * @file    shoot_scheduler.h
 * @brief   Scheduler to control shooter to meet the target, including a thread to invoke PID calculation in period.
 *
 * @addtogroup shoot
 * @{
 */

#ifndef META_INFANTRY_SHOOT_SCHEDULER_H
#define META_INFANTRY_SHOOT_SCHEDULER_H

#include "ch.hpp"

#include "can_motor_controller.h"
#include "pid_controller.hpp"

/**
 * @name ShootSKD
 * @note SKD stands for "scheduler"
 * @brief Scheduler to control shooter to meet the target, including a thread to invoke PID calculation in period.
 * @pre GimbalIF and ChassisIF have been initialized properly
 * @pre GimbalSKD has started, since this SKD relies on GimbalSKD to call GimbalIF::clip_gimbal_current()
 * @usage 1. start(), load_pid_params()
 *        2. set_mode() and set_target_angle() as needed
 *        3. Leave the rest of the work to this SKD
 * @note ABOUT COORDINATE
 *       All components in this SKD use gimbal coordinate, including targets and PID controllers. Only when reading
 *       from or writing to GimbalIF will coordinate transform (by multiple to install_direction_t) be performed based
 *       on yaw_install and pitch_install
 * @note ABOUT SHOOT SPEED
 *       Shoot speed is related to two things: angle (about shoot number) and desired velocity (about shoot speed).
 *       Consequently, special a2v PIDs are applied here, with large kp and out_limit as desired velocity. Since kp is
 *       large, when there are bullets to shoot, output can reach out_limit (desired velocity) easily. When there is no
 *       bullet to shoot, even if kp is large, difference between target angle and actual angle is small, so output
 *       velocity can still reach 0.
 * @note To avoid ShootLG (logic level) to access GimbalIF (interface level) over scheduler level, this module provides
 *       lots of by-pass function to access GimbalIF, with coordinate changes
 */
class ShootSKD {

public:

    enum mode_t {
        FORCED_RELAX_MODE,       // zero force (Still taking control of ChassisIF. External writing to target currents
                                 // will leads to conflicts.) Not affecting friction wheels
        LIMITED_SHOOTING_MODE    // using angle control to shoot specific number of bullet
    };

    /**
     * Start this scheduler.
     * @param loader_install_            Installation direction of loader
     * @param thread_prio                Priority of PID calculating thread
     */
    static void start(tprio_t thread_prio);

    /**
     * Set mode of this SKD
     * @param skd_mode
     */
    static void set_mode(mode_t skd_mode);

    /**
     * Get mode of this SKD
     * @return Current mode
     */
    static mode_t get_mode();

    /**
     * Set bullet loader target angle (related to shoot NUMBER ) in LIMITED_SHOOTING_MODE
     * @param loader_target_angle   Bullet loader target ACCUMULATED angle [positive, degree]
     */
    static void set_loader_target_angle(float loader_target_angle);

    /**
     * Set bullet loader target velocity (related to shoot SPEED) in LIMITED_SHOOTING_MODE
     * @param degree_per_second   Bullet loader target velocity [positive, degree/s]
     */
    static void set_loader_target_velocity(float degree_per_second);

    /**
     * Set friction wheels duty cycle in LIMITED_SHOOTING_MODE or REVERSE_TURNING_MODE
     * @param velocity  [deg/s]
     */
    static void set_friction_wheels(float velocity);


    /** -------------------------------------- Functions to access GimbalIF -------------------------------------- */

    /**
     * Get friction wheels duty cycle
     * @return Friction wheels duty cycle, from 0 to 1.0
     */
    static float get_friction_wheels_target_velocity();

    /**
     * Get bullet loader target angle
     * @return Bullet loader target angle
     */
    static float get_loader_target_angle();
    /**
     * Get bullet loader accumulated angle
     * @return Bullet loader accumulated angle [positive for normal shooting, degree]
     */
    static float get_loader_accumulated_angle();

    /**
     * Reset accumulated angle of bullet loader to 0
     */
    static void reset_loader_accumulated_angle();
    /// TODO: Re-enable shell commands
//    static void cmdFeedback(void *);
    static const Shell::Command shellCommands[];

private:

    static mode_t mode;
    static float bullet_target_angle;
    static float fw_target_velocity;

    static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skd_thread;
    /// TODO: Re-enable shell commands
/***
    static DECL_SHELL_CMD(cmdInfo);
    static DECL_SHELL_CMD(cmdEnableFeedback);
    static DECL_SHELL_CMD(cmdPID);
    static DECL_SHELL_CMD(cmdEnableMotor);
***/
};

#endif //META_INFANTRY_SHOOT_SCHEDULER_H

/** @} */