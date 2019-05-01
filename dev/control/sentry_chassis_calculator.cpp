//
// Created by zhukerui on 2019/4/29.
//

#include "sentry_chassis_calculator.h"

bool SentryChassisController::enable = false;
bool SentryChassisController::test_mode = true;
SentryChassisController::motor_calculator_t SentryChassisController::motor_calculator[MOTOR_COUNT];


void SentryChassisController::init_controller(CANInterface* can_interface) {
    init(can_interface);
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

void SentryChassisController::motor_calculator_t::print_pid_params(BaseSequentialStream *chp){
    chprintf(chp, "motor_id: %d" SHELL_NEWLINE_STR, id);
    chprintf(chp, "dist_to_v pid  kp = %.2f ki = %.2f kd = %.2f i_limit = %.2f out_limit = %.2f" SHELL_NEWLINE_STR,
             dist_to_v.kp, dist_to_v.ki, dist_to_v.kd, dist_to_v.i_limit, dist_to_v.out_limit);
    chprintf(chp, "v_to_i pid  kp = %.2f ki = %.2f kd = %.2f i_limit = %.2f out_limit = %.2f" SHELL_NEWLINE_STR,
             v_to_i.kp, v_to_i.ki, v_to_i.kd, v_to_i.i_limit, v_to_i.out_limit);
}

int SentryChassisController::motor_calculator_t::set_target_current(){
    if (enable){
        motor[id].target_current = (int)(v_to_i.calc(motor[id].actual_angular_velocity, dist_to_v.calc(present_position, target_position)));
    }else {
        motor[id].target_current = 0;
    }
    return motor[id].target_current;
}

void SentryChassisController::motor_calculator_t::reset_position(){
    motor[id].round_count = 0;
    motor[id].actual_angle = 0;
    motor[id].last_angle_raw = 0;
    present_position = 0;
    target_position = 0;
}

float SentryChassisController::motor_calculator_t::update_position(){
    // Count the rounds first, like 1.5 rounds, -20.7 rounds, etc
    // Then transform it to displacement by multiplying the displacement_per_round factor
    present_position = (motor[id].actual_angle + 8192.0f * motor[id].round_count) / 8192.0f * displacement_per_round;
    return present_position;
}

float SentryChassisController::motor_calculator_t::update_velocity(){
    // The unit of actual_angular_velocity is degrees/s, so we first translate it into r/s and then multiplying by displacement_per_round factor
    present_velocity = motor[id].actual_angular_velocity / 360.0f * displacement_per_round;
    return present_velocity;
}