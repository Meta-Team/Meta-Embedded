//
// Created by liuzikai on 2019-07-15.
//

#include "sentry_chassis_logic.h"

float SChassisSKD::radius;
float SChassisSKD::terminals[6] = {LEFT_END, CURVE_1_LEFT, CURVE_1_RIGHT, CURVE_2_LEFT, CURVE_2_RIGHT, RIGHT_END};
int SChassisSKD::prev_terminal;
int SChassisSKD::next_terminal;
unsigned SChassisSKD::last_attack_time;

void SChassisSKD::init() {
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
    set_target_power(25);
}

void SChassisSKD::set_mode(chassis_mode_t target_mode) {
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

/** When enemies are spotted or the sentry is being attacked, start escaping using POM */
void SChassisSKD::start_escaping(){
    last_attack_time = SYSTIME;
    if (!randomMode){
        randomMode = true;
        update_terminal();
    }
}

void SChassisSKD::update_terminal() {
    if (randomMode){
        int dest = SYSTIME % 6; // get a random index between 0 and 5 and decide the next terminal accordingly

        if (dest == next_terminal) {
            next_terminal = prev_terminal;
            prev_terminal = dest;
        } else {
            if ((dest - next_terminal) * SChassisIF::present_velocity > 0) dest = 5 - dest;
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
    set_destination(terminals[next_terminal]);
    startPOM();
}

/**  stop the randomMode and prepare for cruising */
void SChassisSKD::stop_escaping() {
    randomMode = false;
    // return to cruising mode
    if ( SChassisIF::present_velocity>0 )
        next_terminal = 5;
    else
        next_terminal = 0;
}

void SChassisSKD::update_target_current() {
    if (enable){

        float sentry_present_position = SChassisIF::present_position;

        switch (running_mode) {
            case (ONE_STEP_MODE):
                // If we are in the ONE_STEP_MODE
                if (sentry_present_position >= SChassisIF::target_position - 3 &&
                    sentry_present_position <= SChassisIF::target_position + 3) {
                    // If the sentry is in the "stop area", we stop the sentry by simply set the target velocity to 0
                    SChassisIF::target_velocity = 0;
                } else {
                    SChassisIF::target_velocity = sentry_a2v_pid.calc(sentry_present_position,
                                                                      SChassisIF::target_position);
                }
                break;
            case (SHUTTLED_MODE): {
                // If we are in the SHUTTLED_MODE
                if (sentry_present_position > radius - 3) {
                    set_destination(-radius);
                    startPOM();
                }
                else if (sentry_present_position < -radius + 3) {
                    set_destination(radius);
                    startPOM();
                }

                if (POM) {
                    if ((SChassisIF::present_velocity > 0.8 * CRUISING_SPEED) || (SChassisIF::present_velocity < -0.8 * CRUISING_SPEED))
                        stopPOM();
                    float delta_velocity = sentry_POM_pid.calc(Referee::power_heat_data.chassis_power, SChassisIF::power_limit);
                    if (SChassisIF::target_position > sentry_present_position)
                        SChassisIF::target_velocity = SChassisIF::present_velocity + delta_velocity;
                    else if (SChassisIF::target_position < sentry_present_position)
                        SChassisIF::target_velocity = SChassisIF::present_velocity - delta_velocity;
                } else
                    SChassisIF::target_velocity = sentry_a2v_pid.calc(sentry_present_position, SChassisIF::target_position);

                break; }

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
                    if ((sentry_present_position < terminals[next_terminal] && SChassisIF::present_velocity > 0.8 * CRUISING_SPEED) ||
                        (sentry_present_position > terminals[next_terminal] && SChassisIF::present_velocity < - 0.8 * CRUISING_SPEED))
                        stopPOM();

                    float delta_velocity = sentry_POM_pid.calc(Referee::power_heat_data.chassis_power, SChassisIF::power_limit);
                    if (terminals[next_terminal] > sentry_present_position)
                        // If target position is greater than the present position, then we consider it as a negative direction accelerate
                        SChassisIF::target_velocity = SChassisIF::present_velocity + delta_velocity;
                    else if (terminals[next_terminal] < sentry_present_position)
                        // If target position is greater than the present position, then we consider it as a negative direction accelerate
                        SChassisIF::target_velocity = SChassisIF::present_velocity - delta_velocity;
                    // If target position is the same as present position, then we can not decide and do nothing

                } else {
                    SChassisIF::target_velocity = sentry_a2v_pid.calc(sentry_present_position, terminals[next_terminal]);
                }
                break;
            case (STOP_MODE):
            default:
                // If we are in the STOP_MODE, then the sentry now is not movable
                SChassisIF::target_velocity = 0;
        }
        // Set the target current
        SChassisIF::motor[0].target_current = (int16_t)(right_v2i_pid.calc(SChassisIF::motor[0].motor_present_velocity, SChassisIF::target_velocity));
        SChassisIF::motor[1].target_current = (int16_t)(left_v2i_pid.calc(SChassisIF::motor[1].motor_present_velocity, SChassisIF::target_velocity));

    }else {
        SChassisIF::motor[0].target_current = SChassisIF::motor[1].target_current = 0;
    }
}