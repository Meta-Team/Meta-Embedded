//
// Created by zhukerui on 2019/4/29.
//

#include "sentry_chassis.h"

bool SentryChassisController::enable;
SentryChassisController::sentry_mode_t SentryChassisController::running_mode;
SentryChassisController::motor_calculator_t SentryChassisController::motor_calculator[MOTOR_COUNT];


void SentryChassisController::init_controller(CANInterface* can_interface) {
    init(can_interface);
    enable = false;
    running_mode = STOP_MODE;
    clear_position();
    motor_calculator[MOTOR_LEFT].id = MOTOR_LEFT;
    motor_calculator[MOTOR_RIGHT].id = MOTOR_RIGHT;
}

void SentryChassisController::clear_position() {
    motor_calculator[MOTOR_RIGHT].reset_position();
    motor_calculator[MOTOR_LEFT].reset_position();
}

void SentryChassisController::set_destination(float dist) {
    motor_calculator[MOTOR_RIGHT].set_motor_target_position(dist);
    motor_calculator[MOTOR_LEFT].set_motor_target_position(dist);
}

void SentryChassisController::update_target_current() {
    if(running_mode == STOP_MODE){
        // If we are in the STOP_MODE, then the sentry now is not movable
        motor_calculator[MOTOR_LEFT].set_motor_target_velocity(0);
        motor_calculator[MOTOR_RIGHT].set_motor_target_velocity(0);
    }else if(motor_calculator[MOTOR_LEFT].should_change() || motor_calculator[MOTOR_RIGHT].should_change()){
        // If we are not in the STOP_MODE, that means now the sentry is movable
        // If the sentry is in the "stop area"
        if (running_mode == ONE_STEP_MODE){
            // If we are in the ONE_STEP_MODE, we stop the sentry by simply set the target velocity to 0
            motor_calculator[MOTOR_RIGHT].set_motor_target_velocity(0);
            motor_calculator[MOTOR_LEFT].set_motor_target_velocity(0);
        } else if(running_mode == AUTO_MODE){
            // If we are in the AUTO MODE, we change the destination according to the rule we set in set_auto_destination()
            change_auto_destination();
        }
    }
    motor_calculator[MOTOR_RIGHT].set_target_current();
    motor_calculator[MOTOR_LEFT].set_target_current();
}

void SentryChassisController::change_auto_destination() {
    if(motor_calculator[0].position() > 0){
        motor_calculator[MOTOR_RIGHT].set_motor_target_position(-radius);
        motor_calculator[MOTOR_LEFT].set_motor_target_position(-radius);
    } else{
        motor_calculator[MOTOR_RIGHT].set_motor_target_position(radius);
        motor_calculator[MOTOR_LEFT].set_motor_target_position(radius);
    }
}

void SentryChassisController::motor_calculator_t::set_motor_target_position(float dist) {
    target_position = dist;
    // Every time a new target position is set, a new target velocity should be decided
    if (target_position > present_position){
        target_velocity = maximum_speed;
    } else if (target_position < present_position){
        target_velocity = - maximum_speed;
    } else{
        target_velocity = 0;
    }
}

void SentryChassisController::motor_calculator_t::print_pid_params(){
    LOG("motor_id: %d" SHELL_NEWLINE_STR, id);
    LOG("kp = %.2f ki = %.2f kd = %.2f i_limit = %.2f out_limit = %.2f" SHELL_NEWLINE_STR,
             v_to_i.kp, v_to_i.ki, v_to_i.kd, v_to_i.i_limit, v_to_i.out_limit);
}

void SentryChassisController::motor_calculator_t::set_target_current(){
    if (enable){
        motor[id].target_current = (int)(v_to_i.calc(present_velocity, target_velocity));
    }else {
        motor[id].target_current = 0;
    }
}

void SentryChassisController::motor_calculator_t::reset_position(){
    motor[id].round_count = 0;
    motor[id].actual_angle = 0;
    present_position = 0;
    target_position = 0;
    target_velocity = 0;
}

void SentryChassisController::motor_calculator_t::update_motor_data(){
    // Count the rounds first, like 1.5 rounds, -20.7 rounds, etc
    // Then transform it to displacement by multiplying the displacement_per_round factor
    present_position = (motor[id].actual_angle + 8192.0f * motor[id].round_count) / 8192.0f * displacement_per_round / chassis_motor_decelerate_ratio;
    // The unit of actual_angular_velocity is degrees/s, so we first translate it into r/s and then multiplying by displacement_per_round factor
    present_velocity = motor[id].actual_angular_velocity / 360.0f * displacement_per_round;
}