//
// Created by zhukerui on 2019/4/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define SYSTIME (TIME_I2MS(chVTGetSystemTime()))

#include "sentry_chassis_interface.h"
#include "referee_interface.h"
#include "pid_controller.hpp"
#include "can_interface.h"
#include "ch.hpp"
#include "hal.h"
#include "math.h"

class SentryChassisController: public SentryChassis{
public:

    enum sentry_mode_t{
        STOP_MODE,
        CONST_CURRENT_MODE,
        ONE_STEP_MODE,
        AUTO_MODE
    };

    static bool enable;

    /**
     * @brief initialize the calculator class
     * @param can_interface
     */
    static void init_controller(CANInterface* can_interface);

    /**
     * @brief change the running mode if needed
     * @param target_mode
     */
    static void set_mode(sentry_mode_t target_mode, float index = 0);

    /**
     * @brief set the present position and target position to be the 0 point
     */
    static void clear_position();

    /**
     * @brief set the target position and the target velocity according to the given position
     * @param dist the given position, positive for right, negative for left
     */
    static void set_destination(float dist);

    /**
 * @brief use the present data and PIDController to calculate and set the target current that will be sent
 */
    static void update_target_current();

    /**
     * @brief set the speed mode
     * @param mode true for varying_speed mode, false for const_speed mode
     */
    static void change_speed_mode(bool mode){
        change_speed = mode;
        if(change_speed) start_time = SYSTIME;
    }

    /**
     * @brief change the parameters for the v_to_i PIDController
     */
    static void change_v_to_i_pid(float _kp, float _ki, float _kd, float _i_limit, float _out_limit){

        motor_right_pid.change_parameters({_kp, _ki, _kd, _i_limit, _out_limit});
        motor_right_pid.clear_i_out();
        motor_left_pid.change_parameters({_kp, _ki, _kd, _i_limit, _out_limit});
        motor_left_pid.clear_i_out();
    }

    /**
     * @brief Debug helper function. Print the PIDController parameters
     * @param motor_id
     */
    static void print_pid_params(){
        PIDControllerBase::pid_params_t right = motor_right_pid.get_parameters();
        PIDControllerBase::pid_params_t left = motor_left_pid.get_parameters();
        LOG("motor_right:" SHELL_NEWLINE_STR);
        LOG("kp = %.2f ki = %.2f kd = %.2f i_limit = %.2f out_limit = %.2f" SHELL_NEWLINE_STR,
            right.kp, right.ki, right.kd, right.i_limit, right.out_limit);
        LOG("motor_left:" SHELL_NEWLINE_STR);
        LOG("kp = %.2f ki = %.2f kd = %.2f i_limit = %.2f out_limit = %.2f" SHELL_NEWLINE_STR,
            left.kp, left.ki, left.kd, left.i_limit, left.out_limit);
    }

    /**
     * @brief Debug helper function. Print the present position in cm
     */
    static void print_position(){
        LOG("motor %d position: %.2f", 0, motor[0].present_position);
        LOG("motor %d position: %.2f", 1, motor[1].present_position);
    }

    /**
     * @brief Debug helper function. Print the target current sent to the motors
     */
    static void print_current(){
        LOG("motor %d target_current: %d", 0, motor[0].target_current);
        LOG("motor %d target_current: %d", 1, motor[1].target_current);
    }

    /**
     * @brief Debug helper function. Print the present velocity in cm/s
     */
    static void print_velocity(){
        LOG("motor %d present_velocity: %.2f", 0, motor[0].present_velocity);
        LOG("motor %d present_velocity: %.2f", 1, motor[1].present_velocity);
    }

private:

    static sentry_mode_t running_mode;

    static bool change_speed;

    static time_msecs_t start_time;

    static float target_position;

    static float target_velocity;

    static float radius; // the range that sentry can move around the origin in the AUTO MODE

    static int const_current;

    static PIDController motor_right_pid;
    static PIDController motor_left_pid;

    /**
     * @brief the const values
     */

    static float constexpr maximum_speed = 110.0f;

};


#endif //META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
