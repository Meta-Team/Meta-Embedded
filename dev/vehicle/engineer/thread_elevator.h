//
// Created by liuzikai on 2019-05-18.
//

#ifndef META_INFANTRY_THREAD_ELEVATOR_H
#define META_INFANTRY_THREAD_ELEVATOR_H

#include "ch.hpp"
#include "hal.h"

#include "elevator.h"

class ElevatorThread : public chibios_rt::BaseStaticThread<1024> {

public:

    ElevatorThread(Elevator::pid_params_t ELEVATOR_PID_A2V_PARAMS_,
                   Elevator::pid_params_t ELEVATOR_PID_V2I_PARAMS_)
                   : PID_A2V_PARAMS(ELEVATOR_PID_A2V_PARAMS_),
                     PID_V2I_PARAMS(ELEVATOR_PID_V2I_PARAMS_) {};

    void set_target_height(float target_height_);

private:

    float target_height = 0.0f;

    const Elevator::pid_params_t PID_A2V_PARAMS;
    const Elevator::pid_params_t PID_V2I_PARAMS;

    static constexpr unsigned ELEVATOR_THREAD_INTERVAL = 2; // [ms]

    void main() final;

};

#endif //META_INFANTRY_THREAD_ELEVATOR_H
