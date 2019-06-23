//
// Created by liuzikai on 2019-05-18.
//

#include "thread_elevator.h"

void ElevatorThread::set_target_height(float target_height_) {
    target_height = target_height_;
}


void ElevatorThread::main() {

    setName("elevator");

    Elevator::change_pid_params(PID_A2V_PARAMS, PID_V2I_PARAMS);

    while (!shouldTerminate()) {

        Elevator::calc(target_height);

        Elevator::send_elevator_currents();

        sleep(TIME_MS2I(ELEVATOR_THREAD_INTERVAL));
    }

}