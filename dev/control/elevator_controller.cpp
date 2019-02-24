//
// Created by liuzikai on 2019-02-24.
//

#include "elevator_controller.h"

ElevatorController::status_t ElevatorController::status = ElevatorController::STOP;

float ElevatorController::front_target_position = 0;
float ElevatorController::rear_target_position = 0;

ElevatorController::status_t ElevatorController::get_status() {
    return status;
}

float ElevatorController::get_front_target_position() {
    return front_target_position;
}



void ElevatorController::lift_all_wheels() {
    lift_front_wheels();
    lift_rear_wheels();
}

void ElevatorController::lift_front_wheels() {
    ElevatorInterface::apply_front_position(0);
}

void ElevatorController::lift_rear_wheels() {
    ElevatorInterface::apply_rear_position(0);
}

void ElevatorController::lower_all_wheels() {
    lower_front_wheels();
    lower_rear_wheels();
}

void ElevatorController::lower_front_wheels() {
    ElevatorInterface::apply_front_position(-stage_height);
}

void ElevatorController::lower_rear_wheels() {
    ElevatorInterface::apply_rear_position(-stage_height);
}
