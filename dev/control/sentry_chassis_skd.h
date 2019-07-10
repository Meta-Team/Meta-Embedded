//
// Created by zhukerui on 2019/4/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H

#include "remote_interpreter.h"
#include "referee_interface.h"
#include "sentry_chassis_interface.h"
#include "suspension_gimbal_interface.h"
#include "pid_controller.hpp"
#include "can_interface.h"
#include "vehicle/sentry/vehicle_sentry.h"
#include "ch.hpp"
#include "hal.h"

class SentryChassisSKD{
public:

    enum chassis_mode_t{
        STOP_MODE,
        ONE_STEP_MODE,
        SHUTTLED_MODE,
        V_MODE,
//        POM_MODE,
        FINAL_AUTO_MODE
    };

    // Sentry chassis thread
    class SentryChassisThread: public chibios_rt::BaseStaticThread<512>{
        void main()final;
    };

    static SentryChassisThread sentryChassisThread;

    // Debug options
    static bool printPosition;
    static bool printCurrent;
    static bool printVelocity;
    static bool printPower;

private:

    /**
     * @brief initialize all the necessary parameters of SentryChassisSKD
     */
    static void init();

public:

    /**
     * @brief enable the chassis
     */
    static void turn_on();

    /**
     * @brief disable the chassis
     */
    static void turn_off();

    /**
     * @brief set the pid parameters for PID
     * @param pid_id
     *      0 -- left_v2i_pid, right_v2i_pid
     *      1 -- sentry_a2v_pid
     *      2 -- sentry_POM_pid
     * @param new_params
     */
    static void set_pid(int pid_id, PIDControllerBase::pid_params_t new_params);

    /**
     * @brief set position and velocity to zero
     */
    static void set_origin();

    /**
     * @brief set the chassis mode
     * @param target_mode
     */
    static void set_mode(chassis_mode_t target_mode);

    /**
     * @brief set the target position, bigger position will drive the chassis towards right
     * @param dist the target position
     */
    static void set_destination(float dist);

    /**
     * @brief set the maximum velocity for the chassis during the movement
     * @param new maximum velocity
     * @attention it could be NEGATIVE only if you are in the V_MODE!
     */
    static void set_maximum_velocity(float new_velocity);

    /**
     * @brief set the target power for the chassis
     * @param new power limit for the chassis
     */
     static void set_target_power(float new_power);

    /**
     * Power Optimized Mode ( POM )
     * make the fullest use of the power restriction to accelerate or decelerate
     */

    /**
     * @brief start the Power Optimized Mode
     */
    static void startPOM();

    /**
     * @brief stop the Power Optimized Mode
     */
    static void stopPOM();

    /**
     * @brief start escaping, start the randomMode, change terminal immediately if it is not escaping now
     * @pre Enemies are spotted or the sentry is being attacked
     */
    static void start_escaping();

    /**
     * @brief update the next terminal according to the randomMode (true/false)
     */
    static void update_terminal();

    /**
     * @pre stop escaping and start cruising
     */
    static void stop_escaping();

    /**
     * @brief Debug helper function. Print the present position in cm
     */
    static void print_position(){
        LOG("motor %d position: %.2f", 0, SentryChassisIF::motor[0].motor_present_position);
        LOG("motor %d position: %.2f", 1, SentryChassisIF::motor[1].motor_present_position);
    }

    /**
     * @brief Debug helper function. Print the target current sent to the motors
     */
    static void print_current(){
        LOG("motor %d target_current: %d", 0, SentryChassisIF::motor[0].target_current);
        LOG("motor %d target_current: %d", 1, SentryChassisIF::motor[1].target_current);
    }

    /**
     * @brief Debug helper function. Print the present velocity in cm/s
     */
    static void print_velocity(){
        LOG("motor %d motor_present_velocity: %.2f", 0, SentryChassisIF::motor[0].motor_present_velocity);
        LOG("motor %d motor_present_velocity: %.2f", 1, SentryChassisIF::motor[1].motor_present_velocity);
    }

    /**
     * @brief Debug helper function. Print the present power data from the referee system.
     */
     static void print_power(){
         LOG("chassis power: %.2f", Referee::power_heat_data.chassis_power);
         LOG("chassis power_buffer: %u", Referee::power_heat_data.chassis_power_buffer);
         LOG("chassis current: %u", Referee::power_heat_data.chassis_current);
         LOG("chassis voltage: %u", Referee::power_heat_data.chassis_volt);
     }

    /**
     * @brief debug helper function
     * @param pid_id -- the same as "set_pid"
     */
    static void print_pid(int pid_id){
        if (pid_id == 0){
            PIDControllerBase::pid_params_t to_print = left_v2i_pid.get_parameters();
            LOG("%f %f %f %f %f", to_print.kp, to_print.ki, to_print.kd, to_print.i_limit, to_print.out_limit);
        } else if (pid_id == 1){
            PIDControllerBase::pid_params_t to_print = sentry_a2v_pid.get_parameters();
            LOG("%f %f %f %f %f", to_print.kp, to_print.ki, to_print.kd, to_print.i_limit, to_print.out_limit);
        } else if (pid_id == 2){
            PIDControllerBase::pid_params_t to_print = sentry_POM_pid.get_parameters();
            LOG("%f %f %f %f %f", to_print.kp, to_print.ki, to_print.kd, to_print.i_limit, to_print.out_limit);
        }
    }

private:

    /** Mode Status **/

    // Whether chassis is enabled
    static bool enable;
    // The mode under which chassis is running
    static chassis_mode_t running_mode;
    // Whether chassis is in the Power Optimized Mode [FINAL AUTO MODE ONLY]
    static bool POM;
    // Whether chassis should move randomly [FINAL AUTO MODE ONLY]
    static bool randomMode;

    /** PIDs **/

    static PIDController sentry_a2v_pid;
    static PIDController sentry_POM_pid;
    static PIDController right_v2i_pid;
    static PIDController left_v2i_pid;

    /** Accessory Parameters **/

    // The range that sentry can move around the origin [SHUTTLED MODE ONLY]
    static float radius;

    // Array containing terminals length information [FINAL AUTO MODE ONLY]
    static float terminals[];

    // The index of terminal in the terminals array [FINAL AUTO MODE ONLY]
    static int prev_terminal; // The terminal that chassis is leaving from
    static int next_terminal; // The terminal that chassis is approaching to

    // Record of the last time that an attack is detected [FINAL AUTO MODE ONLY]
    static unsigned last_attack_time;

    /**
     * @brief use the present data and PIDController to calculate and set the target current that will be sent
     */
    static void update_target_current();

};


#endif //META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H