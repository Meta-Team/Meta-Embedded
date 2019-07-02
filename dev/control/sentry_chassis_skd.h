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

    enum sentry_mode_t{
        STOP_MODE,
        ONE_STEP_MODE,
        SHUTTLED_MODE,
        V_MODE,
        FINAL_AUTO_MODE
    };

    class SentryChassisThread: public chibios_rt::BaseStaticThread<512>{
        void main()final;
    };

    static SentryChassisThread sentryChassisThread;

    static bool printPosition;
    static bool printCurrent;
    static bool printVelocity;

    static unsigned last_attack_time;
    static bool randomMode;

//    static float left_terminal;
//    static float right_terminal;

    static float prev_terminal;
    static float next_terminal;

private:
    static void init();

public:
    static void turn_on();

    static void turn_off();

    static void set_pid(int pid_id, PIDControllerBase::pid_params_t new_params);

    static void print_pid(bool print_a2v);

    /** set all to zero: position and velocity for chassis and motors */
    static void set_origin();

    static void set_mode(sentry_mode_t target_mode);

    /**
     * @brief set the target position and the target velocity according to the given position
     * @param dist the given position, positive for right, negative for left
     */
    static void set_destination(float dist);

    static void set_maximum_velocity(float new_velocity);

    /** set target terminals (right and left), and set "present region" to the target region */
//    static void set_terminals(float leftTerminal, float rightTerminal);

    /** Power Optimized Mode, used to accelerate or decelerate quickly, make the fullest use of the power restriction. */
    static void startPOM();

    static void stopPOM();

    /**
     * @pre Enemies are spotted or the sentry is being attacked
     * Escape to the next region that is far away from the enemy, using power optimized mode.
     */
    static void start_escaping();

    static void update_terminal(){
        if (randomMode){
            float dest = terminals[SYSTIME % 6]; // get a random index between 0 and 5 and decide the next terminal accordingly

            if (dest == next_terminal) {
                next_terminal = prev_terminal;
                prev_terminal = dest;
            } else {
                prev_terminal = next_terminal;
                next_terminal = dest;
            }
        } else {
            if (next_terminal == LEFT_END){
                prev_terminal = next_terminal;
                next_terminal = RIGHT_END;
            } else {
                prev_terminal = next_terminal;
                next_terminal = LEFT_END;
            }
        }
        startPOM();
    }

    /**
     * @pre Finish the previous escaping process (arrive at the target region)
     * Exit POM. Prepare for Cruising.
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

private:
    static bool enable;
    static bool POM;
    static float terminals[];
    static PIDController sentry_a2v_pid;
    static PIDController sentry_POM_pid;
    static PIDController right_v2i_pid;
    static PIDController left_v2i_pid;
    static sentry_mode_t running_mode;
    static float radius; // the range that sentry can move around the origin in the SHUTTLED_MODE

    /**
     * @brief use the present data and PIDController to calculate and set the target current that will be sent
     */
    static void update_target_current();

};


#endif //META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H