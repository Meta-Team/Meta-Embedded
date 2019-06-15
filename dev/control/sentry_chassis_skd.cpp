//
// Created by zhukerui on 2019/4/29.
// Modified by jintengjun on 2019/6/11
//

#include "sentry_chassis_skd.h"


/* Parameters*/

SentryChassisSKD::SentryChassisThread SentryChassisSKD::sentryChassisThread;
bool SentryChassisSKD::enable;
PIDController SentryChassisSKD::sentry_a2v_pid;
PIDController SentryChassisSKD::right_v2i_pid;
PIDController SentryChassisSKD::left_v2i_pid;
float SentryChassisSKD::maximum_speed;
bool SentryChassisSKD::printPosition;
bool SentryChassisSKD::printCurrent;
bool SentryChassisSKD::printVelocity;
time_msecs_t SentryChassisSKD::evasive_time;
SentryChassisSKD::sentry_mode_t SentryChassisSKD::running_mode;
float SentryChassisSKD::radius;


/* Functions */

void SentryChassisSKD::init() {
    enable = false;
    sentry_a2v_pid.change_parameters(CRUISING_PID_A2V_PARAMS);
    right_v2i_pid.change_parameters(SENTRY_CHASSIS_PID_V2I_PARAMS);
    left_v2i_pid.change_parameters(SENTRY_CHASSIS_PID_V2I_PARAMS);
    maximum_speed = CRUISING_SPEED;
    printPosition = printCurrent = printVelocity = false;
    running_mode = STOP_MODE;
    radius = 50.0f;
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
        SentryChassisIF::escaping = false;
        SentryChassisIF::hit_detected = false;
        sentry_a2v_pid.change_parameters(CRUISING_PID_A2V_PARAMS);
        sentry_a2v_pid.clear_i_out();
    }
}

void SentryChassisSKD::set_destination(float dist) {
    SentryChassisIF::target_position = dist;
    if (running_mode != FINAL_AUTO_MODE){
        // Every time a new target position is set, a new target velocity should be decided
        if (SentryChassisIF::target_position > SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].motor_present_position){
            SentryChassisIF::target_velocity = maximum_speed;
        } else if (SentryChassisIF::target_position < SentryChassisIF::motor[SentryChassisIF::MOTOR_RIGHT].motor_present_position){
            SentryChassisIF::target_velocity = - maximum_speed;
        } else{
            SentryChassisIF::target_velocity = 0;
        }
    }
    // If it is the FINAL_AUTO_MODE, then the target velocity will be calculated automatically by PIDs
}

void SentryChassisSKD::update_target_current() {

    float sentry_present_position = SentryChassisIF::present_position;

    switch (running_mode){
        case (ONE_STEP_MODE):
            // If we are in the ONE_STEP_MODE
            if(sentry_present_position >= SentryChassisIF::target_position-3 && sentry_present_position <= SentryChassisIF::target_position+3) {
                // If the sentry is in the "stop area", we stop the sentry by simply set the target velocity to 0
                SentryChassisIF::target_velocity = 0;
            } else{
                SentryChassisIF::target_velocity = sentry_a2v_pid.calc(sentry_present_position,SentryChassisIF::target_position);
            }
            break;
        case (SHUTTLED_MODE):
            // If we are in the AUTO MODE
            if(sentry_present_position >= radius-3 || sentry_present_position <= -radius+3) {
                // If the sentry is in the "stop area", we change the destination according to the rule we set in set_auto_destination()
                if(sentry_present_position > 0){
                    set_destination(-radius);
                } else{
                    set_destination(radius);
                }
            }
            break;
        case (V_MODE):
            // this mode is for adjusting velocity pid
            SentryChassisIF::target_velocity = maximum_speed;
            break;
        case (FINAL_AUTO_MODE):
            if (SentryChassisIF::hit_detected && !SentryChassisIF::escaping){
                // If hit is detected but the sentry is not escaping, then it is the first time the hit is detected
                SentryChassisIF::escaping = true; // Set the sentry to be escaping mode
                sentry_a2v_pid.change_parameters(ESCAPE_PID_A2V_PARAMS); // Reset the PID to adept to the new speed mode
                sentry_a2v_pid.clear_i_out();

                if (SentryChassisIF::present_region == STRAIGHTWAY){
                    // In the STRAIGHTWAY, sentry may go to CURVE_1 or CURVE_2, it depends on its present velocity direction
                    if (SentryChassisIF::present_velocity > 0) set_destination(CURVE_2_LEFT);
                    else set_destination(CURVE_1_RIGHT);
                } else if (SentryChassisIF::present_region == CURVE_1){
                    set_destination(CURVE_2_LEFT);
                } else if (SentryChassisIF::present_region == CURVE_2){
                    set_destination(CURVE_1_RIGHT);
                }
            } else if (SentryChassisIF::escaping){
                // If the sentry is escaping and reaches a side of the curve, reset its position_region and a2v PID
                if (sentry_present_position > CURVE_2_LEFT - 3 || sentry_present_position < CURVE_1_RIGHT + 3){
                    SentryChassisIF::escaping = false;
                    SentryChassisIF::hit_detected = false;
                    sentry_a2v_pid.change_parameters(CRUISING_PID_A2V_PARAMS);
                    sentry_a2v_pid.clear_i_out();
                    evasive_time = SYSTIME;
                    if (sentry_present_position > CURVE_2_LEFT - 3){
                        SentryChassisIF::present_region = CURVE_2;
                        set_destination(CURVE_2_RIGHT);
                    } else{
                        SentryChassisIF::present_region = CURVE_1;
                        set_destination(CURVE_1_LEFT);
                    }
                }
                // If the sentry doesn't reach a terminal, then it just keep moving
            } else if (!SentryChassisIF::hit_detected && !SentryChassisIF::escaping) {
                // The same principle of moving as SHUTTLE_MODE
                if (SentryChassisIF::present_region == CURVE_1) {
                    if (sentry_present_position > CURVE_1_LEFT - 3) {
                        if (SYSTIME - evasive_time > 20000) {
                            // If the sentry hasn't been hit for 20s, then it should move back to STRAIGHTWAY
                            set_destination(STRAIGHTWAY_LEFT);
                            SentryChassisIF::present_region = STRAIGHTWAY;
                        } else {
                            set_destination(CURVE_1_RIGHT);
                        }
                    } else if (sentry_present_position < CURVE_1_RIGHT + 3) {
                        set_destination(CURVE_1_LEFT);
                    }
                } else if (SentryChassisIF::present_region == CURVE_2) {
                    if (sentry_present_position < CURVE_2_RIGHT + 3) {
                        if (SYSTIME - evasive_time > 20000) {
                            set_destination(STRAIGHTWAY_RIGHT);
                            SentryChassisIF::present_region = STRAIGHTWAY;
                        } else {
                            set_destination(CURVE_2_LEFT);
                        }
                    } else if (sentry_present_position > CURVE_2_LEFT - 3) {
                        set_destination(CURVE_2_RIGHT);
                    }
                } else {
                    if (sentry_present_position < STRAIGHTWAY_RIGHT + 3) {
                        set_destination(STRAIGHTWAY_LEFT);
                    } else if (sentry_present_position > STRAIGHTWAY_LEFT - 3) {
                        set_destination(STRAIGHTWAY_RIGHT);
                    }
                }
            }
            SentryChassisIF::target_velocity = sentry_a2v_pid.calc(sentry_present_position, SentryChassisIF::target_position);
            break;
        case (STOP_MODE):
        default:
            // If we are in the STOP_MODE, then the sentry now is not movable
            SentryChassisIF::target_velocity = 0;
    }
    // Set the target current
    if (enable){
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