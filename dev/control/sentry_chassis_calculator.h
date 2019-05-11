//
// Created by zhukerui on 2019/4/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define SYSTIME (TIME_I2MS(chVTGetSystemTime()))

#include "sentry_chassis_interface.h"
#include "pid_controller.h"
#include "can_interface.h"
#include "ch.hpp"
#include "hal.h"

class SentryChassisController: public SentryChassis{
public:

    enum sentry_mode_t{
        STOP_MODE,
        ONE_STEP_MODE,
        AUTO_MODE
    };

    static bool enable;

    static float constexpr radius = 30.0f; // the range that sentry can move around the origin in the AUTO MODE

    /**
     * @brief initialize the calculator class
     * @param can_interface
     */
    static void init_controller(CANInterface* can_interface);

    /**
     * @brief change the running mode if needed
     * @param target_mode
     */
    static void set_mode(sentry_mode_t target_mode){
        running_mode = target_mode;
        clear_position();
        if(running_mode == AUTO_MODE){
            set_destination(radius);
        }
    }

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
        if(change_speed) present_time = SYSTIME;
    }

    /**
     * @brief change the parameters for the v_to_i PIDController
     */
    static void change_v_to_i_pid(float _kp, float _ki, float _kd, float _i_limit, float _out_limit){
        for(int i = 0; i < MOTOR_COUNT; i++)
            motor_calculator[i].set_v_to_i_param(_kp, _ki, _kd, _i_limit, _out_limit);
    }

    /**
     * @brief Debug helper function. Print the PIDController parameters
     * @param motor_id
     */
    static void print_pid_params(int motor_id){
        motor_calculator[motor_id].print_pid_params();
    }

    /**
     * @brief update the present position and present velocity information
     */
    static void update_present_data(){
        motor_calculator[MOTOR_RIGHT].update_motor_data();
        motor_calculator[MOTOR_LEFT].update_motor_data();
    }

    /**
     * @brief Debug helper function. Print the present position in cm
     */
    static void print_position(){
        LOG("motor %d position: %.2f", 0, motor_calculator[0].position());
        LOG("motor %d position: %.2f", 1, motor_calculator[1].position());
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
        LOG("motor %d present_velocity: %.2f", 0, motor_calculator[0].velocity());
        LOG("motor %d present_velocity: %.2f", 1, motor_calculator[1].velocity());
    }

private:

    static sentry_mode_t running_mode;

    static bool change_speed;

    static time_msecs_t present_time;

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
         * @attention this function is only used for AUTO MODE
         * @return true if motor should change its target position, false otherwise
         */
        bool should_change(){
            return present_position >= target_position-3 && present_position <= target_position+3;
        }

        /**
         * @brief update the present position and velocity according to the data from interface
         */
        void update_motor_data();

        /**
         * @return present position
         */
        float position() {
            return present_position;
        }
        /**
         * @return present velocity
         */
        float velocity(){
            return present_velocity;
        }

        /**
         * @brief reset the present position and the target position to the origin
         */
        void reset_position();

        /**
         * @brief set the target position and target velocity
         * @param dist (cm)
         */
        void set_motor_target_position(float dist);

        /**
         * set the target velocity
         * @param speed (cm/s)
         */
        void set_motor_target_velocity(float speed){
            target_velocity = speed;
        }

        /**
         * @brief set the v_to_i pid
         */
        void set_v_to_i_param(float _kp, float _ki, float _kd, float _i_limit, float _out_limit){
            v_to_i.change_parameters(_kp, _ki, _kd, _i_limit, _out_limit);
            v_to_i.clear_i_out();
        }

        /**
         * @brief use the PIDs to calculate the target current according to the present position and the target position and the present velocity
         * @return
         */
        void set_target_current();

        /**
         * @brief Debug helper function. Print the PIDController parameters
         */
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

    static float constexpr maximum_speed = 80.0f;
    //todo: determine what displacement is changed as the motor goes one round
    static float constexpr displacement_per_round = 17.28f;

    /**
* @brief set the target destination for auto driving
* change the destination if the two motors are both ready to change target destination, or do nothing change otherwise
* @attention this function is specially for AUTO DRIVING MODE
*/
    static void change_auto_destination();
};


#endif //META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
