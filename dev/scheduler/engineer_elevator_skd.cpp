//
// Created by Kerui Zhu on 7/9/2019.
// Modified by LaiXinyi on 7/19/2019
//

#include "engineer_elevator_skd.h"

float EngineerElevatorSKD::target_height = 0.0f;
float EngineerElevatorSKD::target_location = 0.0f;
float EngineerElevatorSKD::stop_judge_threshold = 2.0f;

EngineerElevatorSKD::operation_t EngineerElevatorSKD::vertical_operation;
EngineerElevatorSKD::operation_t EngineerElevatorSKD::horizontal_operation;

EngineerElevatorSKD::SKDThread EngineerElevatorSKD::skdThread;

void EngineerElevatorSKD::start(tprio_t thread_prio, float time_2_height_ratio_) {
    skdThread.start(thread_prio);
    EngineerElevatorIF::init(time_2_height_ratio_);
}

void EngineerElevatorSKD::set_target_height(float target_height_) {
    if (target_height_ > EngineerElevatorIF::get_current_vertical_location()) {
        vertical_operation = FORWARD;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::VERTICAL, EngineerElevatorIF::FORWARD, 0);
    } else if(target_height_ < EngineerElevatorIF::get_current_vertical_location()) {
        vertical_operation = BACKWARD;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::VERTICAL, EngineerElevatorIF::BACKWARD, 0);
    } else {
        vertical_operation = STOP;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::VERTICAL, EngineerElevatorIF::STOP, 0);
    }
    target_height = target_height_;
}

void EngineerElevatorSKD::set_target_movement(float target_location_) {
    target_location = target_location_;
    if(target_location_ > EngineerElevatorIF::get_current_horizontal_location()) {
        horizontal_operation = FORWARD;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::HORIZONTAL, EngineerElevatorIF::FORWARD, 10);
    } else if(target_location_ < EngineerElevatorIF::get_current_horizontal_location()) {
        horizontal_operation = BACKWARD;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::HORIZONTAL, EngineerElevatorIF::BACKWARD, 10);
    } else {
        horizontal_operation = STOP;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::HORIZONTAL, EngineerElevatorIF::STOP, 10);
    }
    target_location = target_location_;
}

#if ELEVATOR_CLOSE_LOOP_CONTROL
void EngineerElevatorSKD::homing(EngineerElevatorSKD::homing_direction_t homing_direction) {
    if (homing_direction == VERTICAL) {
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::VERTICAL, EngineerElevatorIF::BACKWARD, 5);
    } else {
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::HORIZONTAL, EngineerElevatorIF::BACKWARD, 5);
    }
}
#endif

void EngineerElevatorSKD::SKDThread::main() {
    setName("ElevatorSKD");

    while (!shouldTerminate()) {
#if ELEVATOR_CLOSE_LOOP_CONTROL

#endif
        /// Update motor data
//        if(vertical_operation == FORWARD) {
//            current_height += (SKD_THREAD_INTERVAL * time_2_length_ratio);
//        } else if (vertical_operation == BACKWARD) {
//            current_height -= (SKD_THREAD_INTERVAL * time_2_length_ratio);
//        }
//
//        if(horizontal_operation == FORWARD) {
//            current_location += (SKD_THREAD_INTERVAL * time_2_length_ratio);
//        } else if (horizontal_operation == BACKWARD) {
//            current_location -= (SKD_THREAD_INTERVAL * time_2_length_ratio);
//        }

        /// Operation
        if(ABS_IN_RANGE(target_height - EngineerElevatorIF::get_current_vertical_location(),
                                                                            stop_judge_threshold)) {
            vertical_operation = STOP;
            EngineerElevatorIF::set_elevator(EngineerElevatorIF::VERTICAL, EngineerElevatorIF::STOP, 0);
        }

        if(ABS_IN_RANGE(target_location - EngineerElevatorIF::get_current_horizontal_location(),
                                                                            stop_judge_threshold)) {
            horizontal_operation = STOP;
            EngineerElevatorIF::set_elevator(EngineerElevatorIF::HORIZONTAL, EngineerElevatorIF::STOP, 0);
        }

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));

    }
}