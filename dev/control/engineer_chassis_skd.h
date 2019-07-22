//
// Created by Kerui Zhu on 7/9/2019.
//

#ifndef META_INFANTRY_ENGINEER_CHASSIS_SKD_H
#define META_INFANTRY_ENGINEER_CHASSIS_SKD_H

#include "ch.hpp"
#include "hal.h"

#include "chassis_interface.h"
#include "pid_controller.hpp"
//#include "vehicle/engineer/vehicle_engineer.h"

/**
 * @name EngineerChassisSKD
 * @note +X is right, +Y is front, +w is counter-clockwise
 */
class EngineerChassisSKD : public ChassisBase {

public:

    // Engineer chassis thread
    class EngineerChassisThread: public chibios_rt::BaseStaticThread<512>{
        void main()final;
    };

    static EngineerChassisThread engineerChassisThread;

    static float target_velocity[];

private:
    /**
     * Initialize ChassisInterface and this calculator
     * @param wheel_base
     * @param wheel_tread
     * @param wheel_circumference
     */
    static void init();

public:

    static void lock();

    static void unlock();

    static void set_test_end_time(time_msecs_t run_time);

    /**
     * Change parameters of PID controller of every motor
     * @param pid_params
     */
    static void change_pid_params(PIDControllerBase::pid_params_t pid_params);

    static void print_pid();

    static void set_velocity(float target_vx_, float target_vy_, float target_w_);

    /** rotate about a given wheel, used when one wheel is edged */
    static void pivot_turn(engr_motor_id_t id, float w);

    /**
     * Calculate current for all chassis motors and fill target_velocity[] (in this class) and target_current[]
     * (in ChassisInterface)
     * @param target_vx: target velocity along the x axis with respect to the front of the chassis (mm/s)
     * @param target_vy: target velocity along the y axis with respect to the front of the chassis (mm/s)
     * @param target_w: target angular with respect to the front of the chassis (degree/s, negative value for clockwise)
     */
    static void update_target_current();

private:

    static bool enable;

    static bool time_control;
    static time_msecs_t test_end_time; // [ms]

    static float target_vx;
    static float target_vy;
    static float target_w;

    /**
     * PID controller for each motor
     */
    static PIDController pid[];

    // Angular velocity (degree/s) to velocity (mm/s, based on mechanism structure)
    static float constexpr w_to_v_ratio_ = (CHASSIS_WHEEL_BASE + CHASSIS_WHEEL_TREAD) / 2.0f / 360.0f * 3.14159f;;

    // Wheel speed (mm/s) to wheel angular velocity (degree/s)
    static float constexpr v_to_wheel_angular_velocity_ = 360.0f / CHASSIS_WHEEL_CIRCUMFERENCE;

};


#endif //META_INFANTRY_ENGINEER_CHASSIS_SKD_H
