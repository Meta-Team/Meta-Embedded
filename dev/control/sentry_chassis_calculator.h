//
// Created by zhukerui on 2019/4/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H

#include "sentry_chassis_interface.h"
#include "pid_controller.h"
#include "can_interface.h"

class SentryChassisController: public SentryChassis{
public:
    static bool enable;

    static bool test_mode; // test_mode is true if it is being test and false if it is running automatically

    static float constexpr radius = 30.0f;

    /**
     * @brief initialize the calculator class
     * @param can_interface
     */
    static void init_controller(CANInterface* can_interface);

    /**
     * @brief set the present position to be the 0 point
     */
    static void reset_present_position();

    /**
     * @brief let the sentry move to the given position
     * @param dist the given position, positive for right, negative for left
     */
    static void move_certain_dist(float dist);

    static void change_v_to_i_pid(float _kp, float _ki, float _kd, float _i_limit, float _out_limit){
        for(int i = 0; i < MOTOR_COUNT; i++)
            motor_calculator[i].set_v_to_i_param(_kp, _ki, _kd, _i_limit, _out_limit);
    }

    static void print_pid_params(int motor_id){
        motor_calculator[motor_id].print_pid_params();
    }

    static void update_present_data(){
        motor_calculator[MOTOR_RIGHT].update_motor_data();
        motor_calculator[MOTOR_LEFT].update_motor_data();
    }

    static void print_position(){
        LOG("motor %d position: %.2f", 0, motor_calculator[0].position());
        LOG("motor %d position: %.2f", 1, motor_calculator[1].position());
    }

    static void print_current(){
        LOG("motor %d target_current: %d", 0, motor[0].target_current);
        LOG("motor %d target_current: %d", 1, motor[1].target_current);
    }

    static void print_velocity(){
        LOG("motor %d present_velocity: %.2f", 0, motor_calculator[0].velocity());
        LOG("motor %d present_velocity: %.2f", 1, motor_calculator[1].velocity());
    }

    static void update_target_current();

    static bool should_change_position(){
        return motor_calculator[0].should_change() && motor_calculator[1].should_change();
    }

    /**
     * @brief this function set the rule for the automatically driving
     */
    static void change_position();

private:

    /**
     * @brief process the data for each motor and do the calculation
     * @param unit for displacement: cm
     * @param unit for velocity: cm/s
     */
    class motor_calculator_t{
    public:
        motor_id_t id;

        /**
         * @brief this function decides whether the motor should change its target position
         * @attention this function is only used for autonomous mode
         * @return
         */
        bool should_change();

        /**
         * @brief update the present position and velocity according to the data from interface
         */
        void update_motor_data();

        float position(){
            return present_position;
        }

        float velocity(){
            return present_velocity;
        }

        /**
         * @brief reset the position to the origin
         */
        void reset_position();

        /**
         * @brief set the target position
         * @param dist (cm)
         */
        void set_target_position(float dist);

        /**
         * @brief set the v_to_i pid
         * @param _kp
         * @param _ki
         * @param _kd
         * @param _i_limit
         * @param _out_limit
         */
        void set_v_to_i_param(float _kp, float _ki, float _kd, float _i_limit, float _out_limit){
            v_to_i.change_parameters(_kp, _ki, _kd, _i_limit, _out_limit);
            v_to_i.clear_i_out();
        }

        /**
         * @brief use the PIDs to calculate the target current according to the present position and the target position and the actual velocity
         * @return
         */
        int set_target_current();

        void print_pid_params();

    private:
        float present_position = 0;
        float present_velocity = 0;
        float target_position = 0;
        float target_velocity = 0;
        PIDController v_to_i;
    };

    static motor_calculator_t motor_calculator[];

    /**
     * @brief the const values
     */

    static float constexpr maximum_speed = 100.0f;
    //todo: determine what displacement is changed as the motor goes one round
    static float constexpr displacement_per_round = 1.0f;
};


#endif //META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
