//
// Created by zhukerui on 2019/4/29.
//

#include "sentry_chassis_calculator.h"

bool SentryChassisController::enable;
bool SentryChassisController::test_mode;
SentryChassisController::motor_calculator_t SentryChassisController::motor_calculator[MOTOR_COUNT];


void SentryChassisController::init_controller(CANInterface* can_interface) {
    init(can_interface);
    enable = false;
    test_mode = true;
    reset_present_position();
    motor_calculator[MOTOR_LEFT].id = MOTOR_LEFT;
    motor_calculator[MOTOR_RIGHT].id = MOTOR_RIGHT;
}

void SentryChassisController::reset_present_position() {
    for(int i = 0; i < MOTOR_COUNT; i++)
        motor_calculator[i].reset_position();
}

void SentryChassisController::move_certain_dist(float dist) {
    for(int i = 0; i < MOTOR_COUNT; i++)
        motor_calculator[i].set_target_position(dist);
}

void SentryChassisController::update_target_current() {
    for(int i = 0; i < MOTOR_COUNT; i++)
        motor_calculator[i].set_target_current();
}

void SentryChassisController::change_position() {
    if(motor_calculator[0].position() > 0){
        move_certain_dist(-radius);
    } else{
        move_certain_dist(radius);
    }
    LOG("I hit the end");
}

void SentryChassisController::motor_calculator_t::print_pid_params(){
    LOG("motor_id: %d" SHELL_NEWLINE_STR, id);
    LOG("kp = %.2f ki = %.2f kd = %.2f i_limit = %.2f out_limit = %.2f" SHELL_NEWLINE_STR,
             v_to_i.kp, v_to_i.ki, v_to_i.kd, v_to_i.i_limit, v_to_i.out_limit);
}

int SentryChassisController::motor_calculator_t::set_target_current(){
    if (enable){
        motor[id].target_current = (int)(v_to_i.calc(present_velocity, target_velocity));
    }else {
        motor[id].target_current = 0;
    }
    return motor[id].target_current;
}

void SentryChassisController::motor_calculator_t::reset_position(){
    motor[id].round_count = 0;
    motor[id].actual_angle = 0;
    present_position = 0;
    target_position = 0;
}

void SentryChassisController::motor_calculator_t::set_target_position(float dist) {
    target_position = dist;
    if (target_position > present_position){
        target_velocity = maximum_speed;
    } else if (target_position < present_position){
        target_velocity = - maximum_speed;
    } else{
        target_velocity = 0;
    }
}

void SentryChassisController::motor_calculator_t::update_motor_data(){
    // Count the rounds first, like 1.5 rounds, -20.7 rounds, etc
    // Then transform it to displacement by multiplying the displacement_per_round factor
    present_position = (motor[id].actual_angle + 8192.0f * motor[id].round_count) / 8192.0f * displacement_per_round;
    // The unit of actual_angular_velocity is degrees/s, so we first translate it into r/s and then multiplying by displacement_per_round factor
    present_velocity = motor[id].actual_angular_velocity / 360.0f * displacement_per_round;
}

bool SentryChassisController::motor_calculator_t::should_change() {
    return present_position >= radius-15 || present_position <= -radius+15;
}