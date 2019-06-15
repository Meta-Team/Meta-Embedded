//
// Created by zhukerui on 2019/4/29.
// Modified by jintengjun on 2019/6/11
//

#include "sentry_chassis_skd.h"
#include "referee_interface.h"

bool SentryChassisSKD::enable;
time_msecs_t SentryChassisSKD::evasive_time;
PIDController SentryChassisSKD::sentry_a2v_pid;
PIDController SentryChassisSKD::right_v2i_pid;
PIDController SentryChassisSKD::left_v2i_pid;
SentryChassisSKD::sentry_mode_t SentryChassisSKD::running_mode;
float SentryChassisSKD::target_position;
float SentryChassisSKD::radius;
float SentryChassisSKD::target_velocity;
float SentryChassisSKD::maximum_speed;


void SentryChassisSKD::init_controller(CANInterface* can_interface) {
    SentryChassisIF::init(can_interface);
    enable = false;
    running_mode = STOP_MODE;
    clear_position();
    radius = 50.0f;
    target_velocity = 0.0f;
    maximum_speed = 110.0f;
}

void SentryChassisSKD::clear_position() {
    for(int i = 0; i < SENTRY_CHASSIS_MOTOR_COUNT; i++){
        motor[i].actual_angle = 0;
        motor[i].round_count = 0;
        motor[i].present_position = 0;
        motor[i].present_velocity = 0;
    }
    target_position = 0;
    target_velocity = 0;
}

void SentryChassisSKD::set_destination(float dist) {
    target_position = dist;
    if (running_mode != FINAL_AUTO_MODE){
        // Every time a new target position is set, a new target velocity should be decided
        if (target_position > motor[MOTOR_RIGHT].present_position){
            target_velocity = maximum_speed;
        } else if (target_position < motor[MOTOR_RIGHT].present_position){
            target_velocity = - maximum_speed;
        } else{
            target_velocity = 0;
        }
    }
    // If it is the FINAL_AUTO_MODE, then the target velocity will be calculated automatically by PIDs
}

void SentryChassisSKD::update_target_current() {

    float sentry_present_position = get_sentry_position();

    switch (running_mode){
        case (ONE_STEP_MODE):
            // If we are in the ONE_STEP_MODE
            if(sentry_present_position >= target_position-3 && sentry_present_position <= target_position+3) {
                // If the sentry is in the "stop area", we stop the sentry by simply set the target velocity to 0
                target_velocity = 0;
            } else{
                target_velocity = sentry_a2v_pid.calc(sentry_present_position,target_position);
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
            target_velocity = maximum_speed;
            break;
        case (FINAL_AUTO_MODE):
            if (hit_detected && !escaping){
                // If hit is detected but the sentry is not escaping, then it is the first time the hit is detected
                escaping = true; // Set the sentry to be escaping mode
                sentry_a2v_pid.change_parameters(ESCAPE_A2V_PID_PARAMS); // Reset the PID to adept to the new speed mode
                sentry_a2v_pid.clear_i_out();

                if (present_region == STRAIGHTWAY){
                    // In the STRAIGHTWAY, sentry may go to CURVE_1 or CURVE_2, it depends on its present velocity direction
                    if (get_sentry_velocity() > 0) set_destination(CURVE_2_LEFT);
                    else set_destination(CURVE_1_RIGHT);
                } else if (present_region == CURVE_1){
                    set_destination(CURVE_2_LEFT);
                } else if (present_region == CURVE_2){
                    set_destination(CURVE_1_RIGHT);
                }
            } else if (escaping){
                // If the sentry is escaping and reaches a side of the curve, reset its position_region and a2v PID
                if (sentry_present_position > CURVE_2_LEFT - 3 || sentry_present_position < CURVE_1_RIGHT + 3){
                    escaping = false;
                    hit_detected = false;
                    sentry_a2v_pid.change_parameters(CRUISING_A2V_PID_PARAMS);
                    sentry_a2v_pid.clear_i_out();
                    evasive_time = SYSTIME;
                    if (sentry_present_position > CURVE_2_LEFT - 3){
                        present_region = CURVE_2;
                        set_destination(CURVE_2_RIGHT);
                    } else{
                        present_region = CURVE_1;
                        set_destination(CURVE_1_LEFT);
                    }
                }
                // If the sentry doesn't reach a terminal, then it just keep moving
            } else if (!hit_detected && !escaping) {
                // The same principle of moving as SHUTTLE_MODE
                if (present_region == CURVE_1) {
                    if (sentry_present_position > CURVE_1_LEFT - 3) {
                        if (SYSTIME - evasive_time > 20000) {
                            // If the sentry hasn't been hit for 20s, then it should move back to STRAIGHTWAY
                            set_destination(STRAIGHTWAY_LEFT);
                            present_region = STRAIGHTWAY;
                        } else {
                            set_destination(CURVE_1_RIGHT);
                        }
                    } else if (sentry_present_position < CURVE_1_RIGHT + 3) {
                        set_destination(CURVE_1_LEFT);
                    }
                } else if (present_region == CURVE_2) {
                    if (sentry_present_position < CURVE_2_RIGHT + 3) {
                        if (SYSTIME - evasive_time > 20000) {
                            set_destination(STRAIGHTWAY_RIGHT);
                            present_region = STRAIGHTWAY;
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
            target_velocity = sentry_a2v_pid.calc(sentry_present_position, target_position);
            break;
        case (STOP_MODE):
        default:
            // If we are in the STOP_MODE, then the sentry now is not movable
            target_velocity = 0;
    }
    // Set the target current
    if (enable){
        motor[MOTOR_RIGHT].target_current = (int)(right_v2i_pid.calc(motor[MOTOR_RIGHT].present_velocity, target_velocity));
        motor[MOTOR_LEFT].target_current = (int)(left_v2i_pid.calc(motor[MOTOR_LEFT].present_velocity, target_velocity));

    }else {
        motor[MOTOR_LEFT].target_current = motor[MOTOR_RIGHT].target_current = 0;
    }
}

void SentryChassisSKD::set_mode(sentry_mode_t target_mode) {
    running_mode = target_mode;
    clear_position();
    if(running_mode == SHUTTLED_MODE){
        set_destination(radius);
    } else if (running_mode == FINAL_AUTO_MODE){
        present_region = STRAIGHTWAY;
        escaping = false;
        hit_detected = false;
        sentry_a2v_pid.change_parameters(CRUISING_A2V_PID_PARAMS);
        sentry_a2v_pid.clear_i_out();
    }
}