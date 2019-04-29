//
// Created by admin on 2019/4/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H

#include "sentry_chassis_interface.h"
#include "pid_controller.h"
#include "can_interface.h"

class SentryChassisController: public SentryChassis{
public:
    static bool enable;

    /**
     * @brief initialize the calculator class
     * @param can_interface
     */
    static void init_calculator(CANInterface* can_interface);

    /**
     * @brief set the present position to be the 0 point
     */
    static void reset_present_position();

    /**
     * @brief let the sentry move to the given position
     * @param dist the given position, positive for right, negative for left
     */
    static void move_certain_dist(float dist);

    static void change_dist_to_v_pid(float _kp, float _ki, float _kd, float _i_limit, float _out_limit){
        for(int i = 0; i < MOTOR_COUNT; i++)
            motor_calculator[i].set_dist_to_v_param(_kp, _ki, _kd, _i_limit, _out_limit);
    }

    static void change_v_to_i_pid(float _kp, float _ki, float _kd, float _i_limit, float _out_limit){
        for(int i = 0; i < MOTOR_COUNT; i++)
            motor_calculator[i].set_v_to_i_param(_kp, _ki, _kd, _i_limit, _out_limit);
    }

    static void update_present_data(){
        for(int i = 0; i < MOTOR_COUNT; i++)
            motor_calculator[i].update_position();
    }

private:

    class motor_calculator_t{
    public:
        motor_id_t id;

        void update_position(){
            present_position = (motor[id].actual_angle + motor[id].round_count * 8192) * 360.0f / 8192;
        }

        void reset_position(){
            present_position = 0;
        }

        void set_target_position(float dist){
            target_position = dist;
        }

        void set_dist_to_v_param(float _kp, float _ki, float _kd, float _i_limit, float _out_limit){
            dist_to_v.change_parameters(_kp, _ki, _kd, _i_limit, _out_limit);
            //dist_to_v.clear_i_out();
        }

        void set_v_to_i_param(float _kp, float _ki, float _kd, float _i_limit, float _out_limit){
            v_to_i.change_parameters(_kp, _ki, _kd, _i_limit, _out_limit);
            //v_to_i.clear_i_out();
        }

        void set_target_current(){
            motor[id].target_current = (int)(v_to_i.calc(motor[id].actual_angular_velocity, dist_to_v.calc(present_position, target_position)));
        }
    private:
        float present_position = 0;
        float target_position = 0;
        PIDController dist_to_v;
        PIDController v_to_i;
    };

    static motor_calculator_t motor_calculator[];

    /**
     * @brief the const values
     */

    //todo: the ratio of transition between displacement in meter and angle in rad is to be determined
    static float constexpr displacement_to_angle = 1.0f;
    //todo: the maximum speed is to be determined according to the actual power
    static float constexpr maximum_speed = 20.0f;
};


#endif //META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
