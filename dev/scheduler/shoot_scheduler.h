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

#include "gimbal_interface.h"
#include "pid_controller.hpp"

/**
 * @name ShootSKD
 * @note SKD stands for "scheduler"
 * @brief Scheduler to control shooter to meet the target, including a thread to invoke PID calculation in period.
 * @pre GimbalIF and ChassisIF have been initialized properly
 * @pre GimbalSKD has started, since this SKD relies on GimbalSKD to call GimbalIF::enable_gimbal_current_clip()
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
class ShootSKD : public GimbalBase, public PIDControllerBase {

public:

    enum mode_t {
        FORCED_RELAX_MODE,       // zero force (Still taking control of ChassisIF. External writing to target currents
                                 // will leads to conflicts.) Not affecting friction wheels
        LIMITED_SHOOTING_MODE    // using angle control to shoot specific number of bullet
    };

    enum install_direction_t {
        POSITIVE = 1,
        NEGATIVE = -1
    };

    /**
     * Start this scheduler
     * @param loader_install_            Installation direction of loader
     * @param plate_install_             Installation direction of plate
     * @param thread_prio                Priority of PID calculating thread
     */
    static void start(install_direction_t loader_install_, install_direction_t plate_install_, tprio_t thread_prio);

    /**
     * Set PID parameters of loader and plate
     * @param loader_a2v_params
     * @param loader_v2i_params
     * @param plate_a2v_params
     * @param plate_v2i_params
     */
    static void load_pid_params(pid_params_t loader_a2v_params, pid_params_t loader_v2i_params,
                                pid_params_t plate_a2v_params, pid_params_t plate_v2i_params,
                                pid_params_t fw_left_v2i_params, pid_params_t fw_right_v2i_params);

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
     * Set bullet plate target angle (related to shoot NUMBER ) in LIMITED_SHOOTING_MODE
     * @param plate_target_angle   Bullet plate target ACCUMULATED angle [positive, degree]
     */
    static void set_plate_target_angle(float plate_target_angle);

    /**
     * Set bullet loader target velocity (related to shoot SPEED) in LIMITED_SHOOTING_MODE
     * @param degree_per_second   Bullet loader target velocity [positive, degree/s]
     */
    static void set_loader_target_velocity(float degree_per_second);

    /**
     * Set bullet plate target velocity (related to shoot SPEED) in LIMITED_SHOOTING_MODE
     * @param degree_per_second   Bullet plate target velocity [positive, degree/s]
     */
    static void set_plate_target_velocity(float degree_per_second);

    /**
     * Set friction wheels duty cycle in LIMITED_SHOOTING_MODE or REVERSE_TURNING_MODE
     * @param duty_cycle  Friction wheels duty cycle, from 0 to 1.0
     */
    static void set_friction_wheels(float duty_cycle);


    /** -------------------------------------- Functions to access GimbalIF -------------------------------------- */

    /**
     * Get friction wheels duty cycle
     * @return Friction wheels duty cycle, from 0 to 1.0
     */
    static float get_friction_wheels_duty_cycle();

    /**
     * Get bullet loader target current calculated in this SKD
     * @return Bullet loader target current [positive for normal shooting]
     */
    static int get_loader_target_current();

    /**
     * Get bullet plate target current calculated in this SKD
     * @return Bullet plate target current [positive for normal shooting]
     */
    static int get_plate_target_current();

    /**
     * Get bullet loader actual velocity
     * @return Bullet loader actual velocity [positive for normal shooting]
     */
    static float get_loader_actual_velocity();

    /**
     * Get bullet plate actual velocity
     * @return Bullet plate actual velocity [positive for normal shooting]
     */
    static float get_plate_actual_velocity();

    /**
     * Get bullet loader accumulated angle
     * @return Bullet loader accumulated angle [positive for normal shooting, degree]
     */
    static float get_loader_accumulated_angle();

    /**
     * Get bullet plate accumulated angle
     * @return Bullet plate accumulated angle [positive for normal shooting, degree]
     */
    static float get_plate_accumulated_angle();

    /**
     * Reset accumulated angle of bullet loader to 0
     */
    static void reset_loader_accumulated_angle();

    /**
     * Reset accumulated angle of bullet plate to 0
     */
    static void reset_plate_accumulated_angle();

private:

    static install_direction_t install_position[4];

    static mode_t mode;

    static float target_angle[2];
    static float target_velocity[4];
    static int target_current[4];

    static PIDController v2i_pid[4];
    static PIDController a2v_pid[2];

    static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skdThread;
};

#endif //META_INFANTRY_SHOOT_SCHEDULER_H

/** @} */