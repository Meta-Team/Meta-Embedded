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

#include "ch.hpp"

#include "chassis_interface.h"
#include "gimbal_interface.h"

#include "pid_controller.hpp"
#include "math.h"

/**
 * @name ChassisSKD
 * @note SKD stands for "scheduler"
 * @brief Scheduler to control chassis to meet the target, including a thread to invoke PID calculation in period.
 * @pre GimbalIF and ChassisIF have been initialized properly
 * @usage 1. start(), load_pid_params()
 *        2. set_mode() and set_target() as needed
 *        3. Leave the rest of the work to this SKD
 */
class ChassisSKD : public ChassisBase, public PIDControllerBase {

public:

    enum mode_t {
        STOP_MODE,                // zero force
                                  // No support for chassis coordinate
        GIMBAL_COORDINATE_MODE,   // targets use gimbal coordinates
        PARAM_ADJUST_MODE         // for PID parameter adjustment program
    };

    /**
     * Initialize ChassisInterface and this calculator
     * @param wheel_base            Distance between front axle and the back axle [mm]
     * @param wheel_tread           Distance between left and right wheels [mm]
     * @param wheel_circumference   Circumference of wheels [mm]
     * @param thread_prio           Priority of PID calculation thread
     */
    static void start(float wheel_base, float wheel_tread, float wheel_circumference, tprio_t thread_prio);

    /**
     * Change parameters of PID controller of every motor (shared parameters)
     * @param pid_params
     */
    static void load_pid_params(pid_params_t a2v_pid_params, pid_params_t v2i_pid_params);

    /**
     * Set mode of this SKD
     * @param skd_mode
     */
    static void set_mode(mode_t skd_mode);

    /**
     * Set target values
     * @param vx     Target velocity along the x axis (right) with respect to gimbal coordinate [mm/s]
     * @param vy     Target velocity along the y axis (up) with respect to gimbal coordinate [mm/s]
     * @param theta  Target angle difference between gimbal coordinate and chassis coordinate [degree]
     */
    static void set_target(float vx, float vy, float theta);


private:

    // Local storage
    static mode_t mode;
    static float target_vx;
    static float target_vy;
    static float target_theta;

    static PIDController a2v_pid;               // for theta control
    static PIDController v2i_pid[MOTOR_COUNT];  // speed control for each motoe

    static float target_w;                      // middle value for chassis rotation
    static float target_velocity[MOTOR_COUNT];  // target velocity for each motor (middle value for two-ring PID)
    static int target_current[MOTOR_COUNT];     // local storage of target current of each motor

    // Angular velocity (degree/s) to velocity (mm/s, based on mechanism structure)
    static float w_to_v_ratio_;

    // Wheel speed (mm/s) to wheel angular velocity (degree/s)
    static float v_to_wheel_angular_velocity_;

    // Helper function to convert chassis velocity to velocities of each wheel and perform PID calculation once
    static void velocity_decompose(float vx, float vy, float w);

    static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skdThread;

};

#endif //PROJECT_CHASSIS_CONTROLLER_H

/** @} */
