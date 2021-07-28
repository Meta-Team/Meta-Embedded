//
// Created by Kerui Zhu on 7/9/2019.
// Modified by LaiXinyi on 7/19/2019
//

#include "engineer_elevator_skd.h"

float EngineerElevatorSKD::target_height = 0.0f;
float EngineerElevatorSKD::time_2_length_ratio = 0.0f;
float EngineerElevatorSKD::current_height = 0.0f;
float EngineerElevatorSKD::stop_judge_threshold = 2.0f;

EngineerElevatorSKD::operation_t EngineerElevatorSKD::vertical_operation;
EngineerElevatorSKD::operation_t EngineerElevatorSKD::horizontal_operation;

EngineerElevatorSKD::SKDThread EngineerElevatorSKD::skdThread;

void EngineerElevatorSKD::start(tprio_t thread_prio, float time_2_height_ratio_) {
    skdThread.start(thread_prio);
    time_2_length_ratio = time_2_height_ratio_;
}

float EngineerElevatorSKD::get_current_height() {
    return current_height;
}



void EngineerElevatorSKD::set_target_height(float target_height_) {
    if (target_height_ > current_height) {
        vertical_operation = FORWARD;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::LIFT, EngineerElevatorIF::FORWARD);
    } else if(target_height_ < current_height) {
        vertical_operation = BACKWARD;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::LIFT, EngineerElevatorIF::BACKWARD);
    } else {
        vertical_operation = STOP;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::LIFT, EngineerElevatorIF::STOP);
    }
    target_height = target_height_;
}

void EngineerElevatorSKD::set_target_movement(float target_location_) {
    target_location = target_location_;
    if(target_location_ > current_location) {
        horizontal_operation = FORWARD;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::PUSH, EngineerElevatorIF::FORWARD);
    } else if(target_location_ < current_location) {
        horizontal_operation = BACKWARD;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::PUSH, EngineerElevatorIF::BACKWARD);
    } else {
        horizontal_operation = STOP;
        EngineerElevatorIF::set_elevator(EngineerElevatorIF::PUSH, EngineerElevatorIF::STOP);
    }
    target_location = target_location_;
}

float EngineerElevatorSKD::get_current_movement() {
    return current_location;
}

void EngineerElevatorSKD::SKDThread::main() {
    setName("ElevatorSKD");

    while (!shouldTerminate()){
        /// Update motor data
        if(vertical_operation == FORWARD) {
            current_height += (SKD_THREAD_INTERVAL * time_2_length_ratio);
        } else if (vertical_operation == BACKWARD) {
            current_height -= (SKD_THREAD_INTERVAL * time_2_length_ratio);
        }
        if(horizontal_operation == FORWARD) {
            current_location += (SKD_THREAD_INTERVAL * time_2_length_ratio);
        } else if (horizontal_operation == BACKWARD) {
            current_location -= (SKD_THREAD_INTERVAL * time_2_length_ratio);
        }

        /// Operation
        if(ABS_IN_RANGE(target_height - current_height, stop_judge_threshold)) {
            vertical_operation = STOP;
            EngineerElevatorIF::set_elevator(EngineerElevatorIF::LIFT, EngineerElevatorIF::STOP);
        }
        if(ABS_IN_RANGE(target_location - current_location, stop_judge_threshold)) {
            vertical_operation = STOP;
            EngineerElevatorIF::set_elevator(EngineerElevatorIF::LIFT, EngineerElevatorIF::STOP);
        }
        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}