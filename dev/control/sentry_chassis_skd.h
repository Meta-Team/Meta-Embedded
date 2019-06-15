//
// Created by zhukerui on 2019/4/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H

#define CURVE_1_RIGHT 30.0f
#define CURVE_1_LEFT 40.0f
#define STRAIGHTWAY_RIGHT 50.0f
#define STRAIGHTWAY_LEFT 60.0f
#define CURVE_2_RIGHT 70.0f
#define CURVE_2_LEFT 80.0f
#define CRUISING_SPEED 80.0f
#define ESCAPE_SPEED 110.0f
#define CRUISING_A2V_PID_PARAMS {20, 0, 0, 0, CRUISING_SPEED}
#define ESCAPE_A2V_PID_PARAMS {20, 0, 0, 0, ESCAPE_SPEED}

#include "sentry_chassis_interface.h"
#include "pid_controller.hpp"
#include "can_interface.h"
#include "ch.hpp"
#include "hal.h"
#include "math.h"

class SentryChassisSKD: public SentryChassisIF{
public:

    enum sentry_mode_t{
        STOP_MODE,
        ONE_STEP_MODE,
        SHUTTLED_MODE,
        V_MODE,
        FINAL_AUTO_MODE
    };

    static bool enable;
    static time_msecs_t evasive_time;
    static PIDController sentry_a2v_pid;
    static PIDController right_v2i_pid;
    static PIDController left_v2i_pid;

    /**
     * @brief initialize the calculator class
     * @param can_interface
     */
    static void init_controller(CANInterface* can_interface);

    /**
     * @brief change the running mode if needed
     * @param target_mode
     */
    static void set_mode(sentry_mode_t target_mode);

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

    static void set_maximum_velocity(float new_velocity){
        maximum_speed = new_velocity;
    }

    static float get_target_velocity(){
        return target_velocity;
    }
    static float get_target_position(){
        return target_position;
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

    static float target_position;

    static float target_velocity;

    static float radius; // the range that sentry can move around the origin in the AUTO MODE

    /**
     * @brief the const values
     */

    static float maximum_speed;

};


#endif //META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H