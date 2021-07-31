//
// Created by Kerui Zhu on 7/9/2019.
// Modified by LaiXinyi on 7/19/2019
//

#include "engineer_elevator_skd.h"

float EngineerElevatorSKD::target_height = 0.0f;
float EngineerElevatorSKD::stop_judge_threshold = 2.0f;
float EngineerElevatorSKD::current_height = 0.0f;

EngineerElevatorSKD::operation_t EngineerElevatorSKD::operation;

EngineerElevatorSKD::SKDThread EngineerElevatorSKD::skdThread;

void EngineerElevatorSKD::start(tprio_t thread_prio, float time_2_height_ratio_) {
    skdThread.start(thread_prio);
}

void EngineerElevatorSKD::set_target_height(float target_height_) {
    if (target_height_ > current_height) {
        operation = UPWARD;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::UP, 1000);
    } else if(target_height_ < current_height) {
        operation = DOWNWARD;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::DOWN, 1000);
    } else {
        operation = STOP;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::STOP, 0);
    }
    target_height = target_height_;
}

#if ELEVATOR_CLOSE_LOOP_CONTROL
void EngineerElevatorSKD::homing(EngineerElevatorSKD::homing_direction_t homing_direction) {
    if (homing_direction == VERTICAL) {
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::DOWN, 5);
    }
}
#endif

void EngineerElevatorSKD::SKDThread::main() {
    setName("ElevatorSKD");

    while (!shouldTerminate()) {
#if ELEVATOR_CLOSE_LOOP_CONTROL

#endif
        /// Update motor data
        if(operation == UPWARD) {
            current_height += 1;
        } else if (operation == DOWNWARD) {
            current_height -= 1;
        }

        /// Operation
        if(ABS_IN_RANGE(target_height - current_height, stop_judge_threshold)) {
            operation = STOP;
            EngineerElevatorIF::set_elevator(EngineerElevatorIF::STOP, 0);
        }

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));

    }
}