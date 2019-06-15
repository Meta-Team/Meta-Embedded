//
// Created by zhukerui on 2019/4/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H

#include "sentry_chassis_interface.h"
#include "pid_controller.hpp"
#include "can_interface.h"
#include "referee_interface.h"
#include "vehicle/sentry/vehicle_sentry.h"
#include "ch.hpp"
#include "hal.h"

class SentryChassisSKD{
public:

    enum sentry_mode_t{
        STOP_MODE,
        ONE_STEP_MODE,
        SHUTTLED_MODE,
        V_MODE,
        FINAL_AUTO_MODE
    };

    /**
     * @brief initialize the calculator class
     * @param can_interface
     */
    static void init();

    /**
     * @brief set the present position and target position to be the 0 point
     */
    static void set_origin();

    /**
     * @brief change the running mode if needed
     * @param target_mode
     */
    static void set_mode(sentry_mode_t target_mode);

    /**
     * @brief set the target position and the target velocity according to the given position
     * @param dist the given position, positive for right, negative for left
     */
    static void set_destination(float dist);

    class SentryChassisThread: public chibios_rt::BaseStaticThread<512>{
        void main(void)final;
    };

    static SentryChassisThread sentryChassisThread;

private:
    static bool enable;
    static time_msecs_t evasive_time;
    static PIDController sentry_a2v_pid;
    static PIDController right_v2i_pid;
    static PIDController left_v2i_pid;
    static sentry_mode_t running_mode;
    static float radius; // the range that sentry can move around the origin in the SHUTTLED_MODE
    static float maximum_speed;

    /**
     * @brief use the present data and PIDController to calculate and set the target current that will be sent
     */
    static void update_target_current();

    static void set_maximum_velocity(float new_velocity){
        maximum_speed = new_velocity;
    }

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

};


#endif //META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H