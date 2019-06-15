//
// Created by liuzikai on 2019-05-01.
//

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
 * @pre GimbalSKD has started, since this SKD relies on GimbalSKD to call GimbalIF::send_gimbal_currents()
 * @usage 1. start(), load_pid_params()
 *        2. set_mode() and set_target_angle() as needed
 *        3. Leave the rest of the work to this SKD
 * @note ABOUT COORDINATE
 *       All components in this SKD use gimbal coordinate, including targets and PID controllers. Only when reading
 *       from or writing to GimbalIF will coordinate transform (by multiple to install_direction_t) be performed based
 *       on yaw_install and pitch_install
 */
class ShootSKD : public GimbalBase, public PIDControllerBase {

public:

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
                                pid_params_t plate_a2v_params, pid_params_t plate_v2i_params);

    /**
     * Set buller loader target angle
     * @param loader_target_angle
     */
    static void set_loader_target(float loader_target_angle);

    /**
     * Set bullet plate target angle
     * @param plate_target_angle
     */
    static void set_plate_target(float plate_target_angle);

    /**
     * Set friction wheel duty cycle
     * @param duty_cycle from 0 to 1
     */
    static void set_friction_wheels(float duty_cycle);


private:

    static install_direction_t install_position[2];

    static float target_angle[2];
    static float target_velocity[2];
    static int target_current[2];
    static float target_fw;

    static PIDController v2i_pid[2];
    static PIDController a2v_pid[2];

    static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skdThread;
};


#endif //META_INFANTRY_SHOOT_SCHEDULER_H
