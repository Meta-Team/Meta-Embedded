//
// Created by zhukerui on 2019/4/29.
// Modified by jintengjun on 2019/6/11
// Modified by Laixinyi on 2019/7/1
//

#include "sentry_chassis_skd.h"


/* Parameters */

SentryChassisSKD::SentryChassisThread SentryChassisSKD::sentryChassisThread;

bool SentryChassisSKD::printPosition;
bool SentryChassisSKD::printCurrent;
bool SentryChassisSKD::printVelocity;
bool SentryChassisSKD::printPower;

bool SentryChassisSKD::enable;
SentryChassisSKD::chassis_mode_t SentryChassisSKD::running_mode;
bool SentryChassisSKD::POM;
bool SentryChassisSKD::randomMode;

PIDController SentryChassisSKD::sentry_a2v_pid;
PIDController SentryChassisSKD::sentry_POM_pid;
PIDController SentryChassisSKD::right_v2i_pid;
PIDController SentryChassisSKD::left_v2i_pid;

float SentryChassisSKD::radius;
float SentryChassisSKD::terminals[6] = {LEFT_END, CURVE_1_LEFT, CURVE_1_RIGHT, CURVE_2_LEFT, CURVE_2_RIGHT, RIGHT_END};
int SentryChassisSKD::prev_terminal;
int SentryChassisSKD::next_terminal;
unsigned SentryChassisSKD::last_attack_time;


/* Functions */

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

        if(printPower)
            print_power();

        SentryChassisIF::send_currents();
        sleep(TIME_MS2I(100));
    }
}

void SentryChassisSKD::init() {
    printPosition = printCurrent = printVelocity = false;
    enable = false;
    running_mode = STOP_MODE;
    POM = false;
    randomMode = false;
    set_pid(2, POM_PID_P2V_PARAMS);
    set_pid(1, CRUISING_PID_A2V_PARAMS);
    set_pid(0, SENTRY_CHASSIS_PID_V2I_PARAMS);
    radius = 50.0f;
    prev_terminal = 0;
    next_terminal = 5;
    last_attack_time = 0;
}

void SentryChassisSKD::turn_on(){
    enable = true;
}

void SentryChassisSKD::turn_off(){
    enable = false;
}

void SentryChassisSKD::set_pid(int pid_id, PIDControllerBase::pid_params_t new_params){
    if (pid_id == 0){
        left_v2i_pid.change_parameters(new_params);
        right_v2i_pid.change_parameters(new_params);
        left_v2i_pid.clear_i_out();
        right_v2i_pid.clear_i_out();
    } else if (pid_id == 1) {
        sentry_a2v_pid.change_parameters(new_params);
        sentry_a2v_pid.clear_i_out();
    } else if (pid_id == 2){
        sentry_POM_pid.change_parameters(new_params);
        sentry_POM_pid.clear_i_out();
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

void SentryChassisSKD::set_mode(chassis_mode_t target_mode) {
    running_mode = target_mode;
    set_origin();
    if(running_mode == SHUTTLED_MODE){
        set_destination(radius);
    } else if (running_mode == FINAL_AUTO_MODE){
        prev_terminal = 0;
        next_terminal = 5;
        randomMode = false;
        startPOM();
    }
}

void SentryChassisSKD::set_destination(float dist) {
    SentryChassisIF::target_position = dist;
}

void SentryChassisSKD::set_maximum_velocity(float new_velocity){
    SentryChassisIF::target_velocity = new_velocity;
    set_pid(1, {SENTRY_CHASSIS_PID_A2V_KP, SENTRY_CHASSIS_PID_A2V_KI, SENTRY_CHASSIS_PID_A2V_KD, SENTRY_CHASSIS_PID_A2V_I_LIMIT, new_velocity});
}

void SentryChassisSKD::set_target_power(float new_power) {
    SentryChassisIF::power_limit = new_power;
    set_pid(2, {SENTRY_CHASSIS_PID_P2V_KP, SENTRY_CHASSIS_PID_P2V_KI, SENTRY_CHASSIS_PID_P2V_KD, SENTRY_CHASSIS_PID_P2V_I_LIMIT, new_power});
}

void SentryChassisSKD::startPOM() {
    POM = true;
    sentry_POM_pid.clear_i_out();
}

void SentryChassisSKD::stopPOM() {
    POM = false;
    sentry_a2v_pid.clear_i_out();
}

/** When enemies are spotted or the sentry is being attacked, start escaping using POM */
void SentryChassisSKD::start_escaping(){
    last_attack_time = SYSTIME;
    if (!randomMode){
        randomMode = true;
        update_terminal();
    }
}

void SentryChassisSKD::update_terminal() {
        if (randomMode){
            int dest = SYSTIME % 6; // get a random index between 0 and 5 and decide the next terminal accordingly

            if (dest == next_terminal) {
                next_terminal = prev_terminal;
                prev_terminal = dest;
            } else {
                if ((dest - next_terminal) * SentryChassisIF::present_velocity > 0) dest = 5 - dest;
                prev_terminal = next_terminal;
                next_terminal = dest;
            }
        } else {
            if (next_terminal == 0){
                prev_terminal = next_terminal;
                next_terminal = 5;
            } else {
                prev_terminal = next_terminal;
                next_terminal = 0;
            }
        }
        startPOM();
    }

/**  stop the randomMode and prepare for cruising */
void SentryChassisSKD::stop_escaping() {
    randomMode = false;
    // return to cruising mode
    if ( SentryChassisIF::present_velocity>0 )
        next_terminal = 5;
    else
        next_terminal = 0;
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
                    SentryChassisIF::target_velocity = sentry_a2v_pid.calc(sentry_present_position,
                                                                           SentryChassisIF::target_position);
                }
                break;
            case (SHUTTLED_MODE):
                // If we are in the SHUTTLED_MODE
                if (sentry_present_position > radius - 3) set_destination(-radius);
                else if (sentry_present_position < -radius + 3) set_destination(radius);
                SentryChassisIF::target_velocity = sentry_a2v_pid.calc(sentry_present_position,
                                                                       SentryChassisIF::target_position);
                break;
            case (V_MODE):
                // this mode is for adjusting velocity pid
                // The target velocity is given by user, here we do no calculation to target velocity
                break;
            case (FINAL_AUTO_MODE):
                // If we are in the FINAL_AUTO_MODE
                if ( randomMode && SYSTIME - last_attack_time > 60000) stop_escaping();

                if (sentry_present_position > terminals[next_terminal] - 3 && sentry_present_position < terminals[next_terminal] + 3) update_terminal();

                // POM = false; // Delete this line if referee works

                if ( POM ){
                    if ((sentry_present_position < terminals[next_terminal] && SentryChassisIF::present_velocity > 0.8 * CRUISING_SPEED) ||
                            (sentry_present_position > terminals[next_terminal] && SentryChassisIF::present_velocity < - 0.8 * CRUISING_SPEED))
                        stopPOM();

                    float delta_velocity = sentry_POM_pid.calc(Referee::power_heat_data.chassis_power, SentryChassisIF::power_limit);
                    if (terminals[next_terminal] > sentry_present_position)
                        // If target position is greater than the present position, then we consider it as a negative direction accelerate
                        SentryChassisIF::target_velocity = SentryChassisIF::present_velocity - delta_velocity;
                    else if (terminals[next_terminal] < sentry_present_position)
                        // If target position is greater than the present position, then we consider it as a negative direction accelerate
                        SentryChassisIF::target_velocity = SentryChassisIF::present_velocity + delta_velocity;
                        // If target position is the same as present position, then we can not decide and do nothing

                } else {
                    SentryChassisIF::target_velocity = sentry_a2v_pid.calc(sentry_present_position, terminals[next_terminal]);
                }
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