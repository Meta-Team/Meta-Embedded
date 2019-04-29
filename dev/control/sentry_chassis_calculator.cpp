//
// Created by admin on 2019/4/29.
//

#include "sentry_chassis_calculator.h"

bool SentryChassisController::enable = false;
SentryChassisController::motor_calculator_t SentryChassisController::motor_calculator[MOTOR_COUNT];


void SentryChassisController::init_calculator(CANInterface* can_interface) {
    init(can_interface);
    reset_present_position();
    motor_calculator[MOTOR_LEFT].id = MOTOR_LEFT;
    motor_calculator[MOTOR_RIGHT].id = MOTOR_RIGHT;
}

void SentryChassisController::reset_present_position() {
    for(int i = 0; i < MOTOR_COUNT; i++)
        motor_calculator[i].reset_position();
}

void SentryChassisController::move_certain_dist(float dist) {
    for(int i = 0; i < MOTOR_COUNT; i++){
        motor_calculator[i].set_target_position(dist);
        motor_calculator[i].set_target_current();
    }
}
