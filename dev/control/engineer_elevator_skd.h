//
// Created by Kerui Zhu on 7/9/2019.
//

#ifndef META_INFANTRY_ENGINEER_ELEVATOR_SKD_H
#define META_INFANTRY_ENGINEER_ELEVATOR_SKD_H

#include "ch.hpp"
#include "hal.h"
#include "common_macro.h"
#include "vehicle/engineer/vehicle_engineer.h"
#include "engineer_elevator_interface.h"
#include "pid_controller.hpp"


/**
 * @note positive height - chassis lift up
 */
class EngineerElevatorSKD {

public:

    // Engineer elevator chassis thread
    class EngineerElevatorThread: public chibios_rt::BaseStaticThread<512>{
        void main()final;
    };

    static EngineerElevatorThread engineerElevatorThread;

private:

    static void init();

public:

    static void elevator_enable(bool enable);

    static void aided_motor_enable(bool enable);

    static void change_pid_params(int pid_id, PIDControllerBase::pid_params_t pid_params);

    static void set_target_height(float new_height);

    static void set_aided_motor_velocity(float target_velocity_L, float target_velocity_R);

    static void update_target_current();

    static float target_height;

    // Size: 4; index 0 and 1 for elevator motors, index 2 and 3 for auxiliary motors
    static float target_velocity[];

private:

    static bool elevator_enabled;

    static bool aided_motor_enabled;

    // Size: 4; index 0 and 1 for elevator motors, index 2 and 3 for auxiliary motors
    static PIDController v2i_pid[];

    // Size: 2; index 0 and 1 for elevator motors
    static PIDController a2v_pid[];

    static PIDController counter_balance_pid;

    friend void cmd_elevator_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]);
};


#endif //META_INFANTRY_ENGINEER_ELEVATOR_SKD_H
