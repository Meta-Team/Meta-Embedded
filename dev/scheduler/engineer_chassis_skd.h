//
// Created by Kerui Zhu on 7/9/2019.
//

#ifndef META_INFANTRY_ENGINEER_CHASSIS_SKD_H
#define META_INFANTRY_ENGINEER_CHASSIS_SKD_H

#include "ch.hpp"
#include "hal.h"

#include "chassis_interface.h"
#include "pid_controller.hpp"

/**
 * @name EngineerChassisSKD
 * @note +X is right, +Y is front, +w is counter-clockwise
 */
class EngineerChassisSKD : public ChassisBase {

public:

    /**
     * Initialize ChassisInterface and this calculator
     * @param wheel_base              Distance between front axle and the back axle [mm]
     * @param wheel_tread             Distance between left and right wheels [mm]
     * @param wheel_circumference     Circumference of wheels [mm]
     * @param thread_prio             Priority of PID calculation thread
     */
    static void start(float wheel_base, float wheel_tread, float wheel_circumference, tprio_t thread_prio);

    /**
     * Change PID parameters of PID controller
     * @param v2i_pid_params       Velocity to current parameters of every motor (shared parameters)
     */
    static void load_pid_params(PIDController::pid_params_t v2i_pid_params);

    static void enable(bool enable_);

    static bool is_locked();

    static void set_velocity(float target_vx_, float target_vy_, float target_w_);

    /**
     * Set target values of turning around an wheel
     * @param id   Motor id as pivot
     * @param w    Angular velocity
     */
    static void pivot_turn(float x, float y, float w);

private:

    static bool enabled;

    static float target_vx;
    static float target_vy;
    static float target_w;

    static float target_velocity[MOTOR_COUNT];

    static PIDController pid[MOTOR_COUNT];

    static float wheel_base_;  // distance between front axle and the back axle [mm]
    static float wheel_tread_;  // distance between left and right wheels [mm]
    static float wheel_circumference_;  // circumference of wheels [mm]

    static float w_to_v_ratio_;  // Angular velocity (degree/s) to velocity (mm/s, based on mechanism structure)
    static float v_to_wheel_angular_velocity_;  // Wheel speed (mm/s) to wheel angular velocity (degree/s)

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        ///about the constexpr: the compiler will check if variable was indeed assigned a constant value
        static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]
        ///about the final: the derived classes cannot override this main function
        void main() final;
    };

    static SKDThread skdThread;

    friend class EngineerFeedbackThread;

};


#endif //META_INFANTRY_ENGINEER_CHASSIS_SKD_H
