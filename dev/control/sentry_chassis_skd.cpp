//
// Created by zhukerui on 2019/4/29.
// Modified by jintengjun on 2019/6/11
//

#include "sentry_chassis_skd.h"


/* Parameters */

SentryChassisSKD::SentryChassisThread SentryChassisSKD::sentryChassisThread;
bool SentryChassisSKD::enable;
bool SentryChassisSKD::POM;
PIDController SentryChassisSKD::sentry_calcv_pid;
PIDController SentryChassisSKD::right_v2i_pid;
PIDController SentryChassisSKD::left_v2i_pid;
bool SentryChassisSKD::printPosition;
bool SentryChassisSKD::printCurrent;
bool SentryChassisSKD::printVelocity;
SentryChassisSKD::sentry_mode_t SentryChassisSKD::running_mode;
float SentryChassisSKD::radius;
float SentryChassisSKD::left_terminal;
float SentryChassisSKD::right_terminal;


/* Functions */

void SentryChassisSKD::init() {
    enable = false;
    set_pid(true, CRUISING_PID_A2V_PARAMS);
    set_pid(false, SENTRY_CHASSIS_PID_V2I_PARAMS);
    printPosition = printCurrent = printVelocity = false;
    running_mode = STOP_MODE;
    radius = 50.0f;
}

void SentryChassisSKD::turn_on(){
    enable = true;
}

void SentryChassisSKD::turn_off(){
    enable = false;
}

void SentryChassisSKD::set_pid(bool change_a2v, PIDControllerBase::pid_params_t new_params){
    if (change_a2v) {
        sentry_calcv_pid.change_parameters(new_params);
        sentry_calcv_pid.clear_i_out();
    } else{
        left_v2i_pid.change_parameters(new_params);
        right_v2i_pid.change_parameters(new_params);
        left_v2i_pid.clear_i_out();
        right_v2i_pid.clear_i_out();
    }
}

void SentryChassisSKD::print_pid(bool print_a2v){
    if (print_a2v){
        PIDControllerBase::pid_params_t to_print = sentry_calcv_pid.get_parameters();
        LOG("%f %f %f %f %f", to_print.kp, to_print.ki, to_print.kd, to_print.i_limit, to_print.out_limit);
    } else{
        PIDControllerBase::pid_params_t to_print = right_v2i_pid.get_parameters();
        LOG("%f %f %f %f %f", to_print.kp, to_print.ki, to_print.kd, to_print.i_limit, to_print.out_limit);
    }
}

void SentryChassisSKD::set_origin() {
    SentryChassisIF::motor[0].clear_position();
    SentryChassisIF::motor[1].clear_position();
    SentryChassisIF::target_position = 0;
    SentryChassisIF::target_velocity = 0;
    SentryChassisIF::present_position = 0;
    SentryChassisIF::present_velocity = 0;
}

void SentryChassisSKD::set_mode(sentry_mode_t target_mode) {
    running_mode = target_mode;
    set_origin();
    if(running_mode == SHUTTLED_MODE){
        set_destination(radius);
    } else if (running_mode == FINAL_AUTO_MODE){
        SentryChassisIF::present_region = STRAIGHTWAY;
        sentry_calcv_pid.change_parameters(CRUISING_PID_A2V_PARAMS);
        sentry_calcv_pid.clear_i_out();
        set_terminals(STRAIGHTWAY_LEFT, STRAIGHTWAY_RIGHT);
    }
}

void SentryChassisSKD::set_destination(float dist) {
    SentryChassisIF::target_position = dist;
}

void SentryChassisSKD::update_target_current() {
    if (enable){

        float sentry_present_position = SentryChassisIF::present_position;

        switch (running_mode) {
            case (ONE_STEP_MODE):
                // If we are in the ONE_STEP_MODE
                if (sentry_present_position >= SentryChassisIF::target_position - 3 &&
                    sentry_present_position <= SentryChassisIF::target_position + 3) {
                    // If the sentry is in the "stop area", we stop the sentry by simply set the target velocity to 0
                    SentryChassisIF::target_velocity = 0;
                } else {
                    SentryChassisIF::target_velocity = sentry_calcv_pid.calc(sentry_present_position,
                                                                           SentryChassisIF::target_position);
                }
                break;
            case (SHUTTLED_MODE):
                // If we are in the SHUTTLED_MODE
                if (sentry_present_position > radius - 3) set_destination(-radius);
                else if (sentry_present_position < -radius + 3) set_destination(radius);
                SentryChassisIF::target_velocity = sentry_calcv_pid.calc(sentry_present_position,
                                                                       SentryChassisIF::target_position);
                break;
            case (V_MODE):
                // this mode is for adjusting velocity pid
                // The target velocity is given by user, here we do no calculation to target velocity
                break;
            case (FINAL_AUTO_MODE):
                // If we are in the FINAL_AUTO_MODE
                if (sentry_present_position > right_terminal - 3) set_destination(left_terminal);
                else if (sentry_present_position < left_terminal + 3) set_destination(right_terminal);
                SentryChassisIF::target_velocity = sentry_calcv_pid.calc(sentry_present_position, SentryChassisIF::target_position);
                break;
            case (STOP_MODE):
            default:
                // If we are in the STOP_MODE, then the sentry now is not movable
                SentryChassisIF::target_velocity = 0;
        }
        // Set the target current
        SentryChassisIF::motor[0].target_current = (int16_t)(right_v2i_pid.calc(SentryChassisIF::motor[0].motor_present_velocity, SentryChassisIF::target_velocity));
        SentryChassisIF::motor[1].target_current = (int16_t)(left_v2i_pid.calc(SentryChassisIF::motor[1].motor_present_velocity, SentryChassisIF::target_velocity));

    }else {
        SentryChassisIF::motor[0].target_current = SentryChassisIF::motor[1].target_current = 0;
    }
}

void SentryChassisSKD::SentryChassisThread::main() {

    setName("SentryChassis");
    SentryChassisSKD::init();

    while (!shouldTerminate()){
        update_target_current();

        if(printPosition)
            print_position();

        if(printCurrent)
            print_current();

        if(printVelocity)
            print_velocity();

        SentryChassisIF::send_currents();
        sleep(TIME_MS2I(100));
    }
}