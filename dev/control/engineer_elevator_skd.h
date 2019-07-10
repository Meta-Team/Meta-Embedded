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

    static void elevator_lock(bool enable_);

    static void aided_motor_enable(bool enable);

    static void change_pid_params(int pid_id, PIDControllerBase::pid_params_t pid_params);

    static void update_target_current();

    static constexpr float ANGLE_HEIGHT_RATIO = 458.0f;  // [degree/cm]

    // Size: 2; index 0 for elevator motors, index 1 for auxiliary motors
    static float target_velocity[];

private:

    static bool elevator_enabled;

    static bool elevator_locked;

    static bool aided_motor_enabled;

    static float target_height;


    // Size: 4; index 0 and 1 for elevator motors, index 2 and 3 for auxiliary motors
    static PIDController v2i_pid[];

    static PIDController counter_balance_pid;
};


#endif //META_INFANTRY_ENGINEER_ELEVATOR_SKD_H
