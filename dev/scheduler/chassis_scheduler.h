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

#include <logic/chassis_logic.h>
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

    enum install_mode_t {
        POSITIVE = 1,
        NEGATIVE = -1
    };

    enum mode_t {
        FORCED_RELAX_MODE,         // zero force (Still taking control of ChassisIF. External writing to target currents
                                   // will leads to conflicts.)
                                   // NO SUPPORT FOR CHASSIS COORDINATE
        GIMBAL_COORDINATE_MODE,    // targets use gimbal coordinates

        ANGULAR_VELOCITY_DODGE_MODE  // input a constant angular velocity, to rotate.
    };

    /**
     * Initialize ChassisInterface and this calculator
     * @param wheel_base              Distance between front axle and the back axle [mm]
     * @param wheel_tread             Distance between left and right wheels [mm]
     * @param wheel_circumference     Circumference of wheels [mm]
     * @param install_mode            Whether chassis motors are reversed with some mechanism
     * @param chassis_gimbal_offset   Distance between gimbal and chassis [mm, + for gimbal at "front"]
     * @param thread_prio             Priority of PID calculation thread
     */
    static void start(float wheel_base, float wheel_tread, float wheel_circumference, install_mode_t install_mode,
                      float chassis_gimbal_offset, tprio_t thread_prio);

    /**
     * Change PID parameters of PID controller
     * @param theta2v_pid_params   Theta (see set_target()) to chassis rotation PID parameters
     * @param v2i_pid_params       Velocity to current parameters of every motor (shared parameters)
     */
    static void load_pid_params(pid_params_t theta2v_pid_params, pid_params_t v2i_pid_params);

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

    /**
     * Set dodge_mode target
     * @param vx    Target velocity along the x axis (right) with respect to gimbal coordinate [mm/s]
     * @param vy    @param vy     Target velocity along the y axis (up) with respect to gimbal coordinate [mm/s]
     * @param omega  Target angle difference between gimbal coordinate and chassis coordinate [degree]
     */
     static void set_dodge_target(float vx, float vy, float omega);

    /**
     * Get actual angle difference between gimbal coordinate and chassis coordinate [degree]
     * @return Theta value
     */
    static float get_actual_theta();

    /**
     * Get v2i PID parameters.
     * @return PID params
     */
    static pid_params_t echo_pid_params();

    /**
     * Get actual velocity
     * @param Motor ID
     * @return motor actual velocity
     */
     static float get_actual_velocity(motor_id_t motorId);

     /**
      * Get target velocity
      * @param Motor ID
      * @return motor target velocity
      */
     static float get_target_velocity(motor_id_t motorId);

private:

    // Local storage
    static mode_t mode;
    static float target_vx;
    static float target_vy;
    static float target_theta;

    static PIDController a2v_pid;               // for theta control
    static PIDController v2i_pid[MOTOR_COUNT];  // speed control for each motor

    static float target_w;                      // middle value for chassis rotation
    static float target_velocity[MOTOR_COUNT];  // target velocity for each motor (middle value for two-ring PID)
    static int target_current[MOTOR_COUNT];     // local storage of target current of each motor

    static float wheel_base_;  // distance between front axle and the back axle [mm]
    static float wheel_tread_;  // distance between left and right wheels [mm]
    static float wheel_circumference_;  // circumference of wheels [mm]

    static float w_to_v_ratio_;  // Angular velocity (degree/s) to velocity (mm/s, based on mechanism structure)
    static float v_to_wheel_angular_velocity_;  // Wheel speed (mm/s) to wheel angular velocity (degree/s)

    static float chassis_gimbal_offset_;  // distance between gimbal and chassis [mm, + for gimbal at "front"]

    static install_mode_t install_mode_;

    static bool sports_mode_on;
    friend void ChassisLG::set_sports_mode();
    friend bool ChassisLG::get_sports_mode();

    // Helper function to convert chassis velocity to velocities of each wheel and perform PID calculation once
    static void velocity_decompose_(float vx, float vy, float w);

    static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skdThread;

};

#endif //PROJECT_CHASSIS_CONTROLLER_H

/** @} */
