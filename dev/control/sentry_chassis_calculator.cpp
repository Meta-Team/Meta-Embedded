//
// Created by zhukerui on 2019/4/29.
//

#include "sentry_chassis_calculator.h"

bool SentryChassisController::enable;
SentryChassisController::sentry_mode_t SentryChassisController::running_mode;
bool SentryChassisController::change_speed;
time_msecs_t SentryChassisController::present_time;
float SentryChassisController::target_position;
float SentryChassisController::target_velocity;
PIDController SentryChassisController::motor_right_pid;
PIDController SentryChassisController::motor_left_pid;


void SentryChassisController::init_controller(CANInterface* can_interface) {
    init(can_interface);
    enable = false;
    running_mode = STOP_MODE;
    clear_position();
    change_speed = false;
}

void SentryChassisController::clear_position() {
    for(int i = 0; i < MOTOR_COUNT; i++){
        motor[i].actual_angle = 0;
        motor[i].round_count = 0;
        motor[i].present_position = 0;
        motor[i].present_velocity = 0;
    }
    target_position = 0;
    target_velocity = 0;
}

void SentryChassisController::set_destination(float dist) {
    target_position = dist;
    // Every time a new target position is set, a new target velocity should be decided
    if (target_position > motor[MOTOR_RIGHT].present_position){
        target_velocity = maximum_speed;
    } else if (target_position < motor[MOTOR_RIGHT].present_position){
        target_velocity = - maximum_speed;
    } else{
        target_velocity = 0;
    }
}

void SentryChassisController::update_target_current() {

    switch (running_mode){
        case (ONE_STEP_MODE):
            // If we are in the ONE_STEP_MODE
            if((motor[MOTOR_LEFT].present_position >= target_position-3 && motor[MOTOR_LEFT].present_position <= target_position+3)
            || (motor[MOTOR_RIGHT].present_position >= target_position-3 && motor[MOTOR_RIGHT].present_position <= target_position+3))
            {
                // If the sentry is in the "stop area", we stop the sentry by simply set the target velocity to 0
                target_velocity = 0;
            }
            break;
        case (AUTO_MODE):
            // If we are in the AUTO MODE
            if((motor[MOTOR_LEFT].present_position >= radius || motor[MOTOR_LEFT].present_position <= -radius)
               || (motor[MOTOR_RIGHT].present_position >= radius || motor[MOTOR_RIGHT].present_position <= -radius)) {
                // If the sentry is in the "stop area", we change the destination according to the rule we set in set_auto_destination()
                if(motor[MOTOR_RIGHT].present_position.position() > 0){
                    set_destination(-radius);
                } else{
                    set_destination(radius);
                }
            }
            break;
        case (STOP_MODE):
        default:
            // If we are in the STOP_MODE, then the sentry now is not movable
            target_velocity = 0;
    }


if(change_speed){
        // If the change_speed is true, then the speed should change variously from time to time

    }
    set_target_current();
}

void SentryChassisController::set_target_current() {
    if (enable){
        motor[MOTOR_RIGHT].target_current = (int)(motor_right_pid.calc(motor[MOTOR_RIGHT].present_velocity, target_velocity));
        motor[MOTOR_LEFT].target_current = (int)(motor_left_pid.calc(motor[MOTOR_LEFT].present_velocity, target_velocity));
    }else {
        motor[MOTOR_LEFT].target_current = motor[MOTOR_RIGHT].target_current = 0;
    }
}