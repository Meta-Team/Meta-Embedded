//
// Created by Kerui Zhu on 7/9/2019.
//

#ifndef META_INFANTRY_ENGINEER_ELEVATOR_SKD_H
#define META_INFANTRY_ENGINEER_ELEVATOR_SKD_H

#include "ch.hpp"
#include "hal.h"
#include "common_macro.h"

#include "engineer_elevator_interface.h"
#include "pid_controller.hpp"


/**
 * @note positive height - chassis lift up
 */
class EngineerElevatorSKD {

public:

    static void start(tprio_t thread_prio);

    static void elevator_enable(bool enable);

    static void aided_motor_enable(bool enable);


    static void load_pid_params(PIDControllerBase::pid_params_t elevator_a2v,
                                PIDControllerBase::pid_params_t elevator_v2i,
                                PIDControllerBase::pid_params_t aided_v2i,
                                PIDControllerBase::pid_params_t balance_a2v);

    // for debug
    static void set_aided_pid_params(PIDControllerBase::pid_params_t aided_v2i);

    static void set_target_height(float new_height);  // [cm], positive height - chassis lift up
    static float get_target_height();

    static void set_aided_motor_velocity(float target_velocity_L, float target_velocity_R);

    // TODO: make it private
    static float target_height;
    static float target_velocity[4];  // 0 and 1 for elevator motors, 2 and 3 for auxiliary motors


private:

    static bool elevator_enabled;
    static bool aided_motor_enabled;



    static PIDController v2i_pid[4];  // 0 and 1 for elevator motors, 2 and 3 for auxiliary motors
    static PIDController a2v_pid[2];  // 0 and 1 for elevator motors
    static PIDController counter_balance_pid;

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]
        void main() final;
    };

    static SKDThread skdThread;

    friend void cmd_elevator_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void cmd_elevator_set_target_velocities(BaseSequentialStream *chp, int argc, char *argv[]);
};


#endif //META_INFANTRY_ENGINEER_ELEVATOR_SKD_H
