//
// Created by Kerui Zhu on 7/11/2019.
//

#include "robotic_arm_skd.h"

RoboticArmSKD::RoboticArmThread RoboticArmSKD::roboticArmThread;
RoboticArmSKD::robotic_arm_state_t RoboticArmSKD::state;
RoboticArmSKD::bullet_state_t RoboticArmSKD::bullet_state;
int RoboticArmSKD::extend_state;
int RoboticArmSKD::clamp_state;
int RoboticArmSKD::slide_x_state;
int RoboticArmSKD::should_set_arm_normal;
bool RoboticArmSKD::released;
float RoboticArmSKD::trigger_angle;
float RoboticArmSKD::target_velocity[2] = {0};
PIDController RoboticArmSKD::v2i_pid[2];


void RoboticArmSKD::start(tprio_t skd_thread_prio) {

    state = NORMAL;
    bullet_state = WAITING;
    released = true;
    v2i_pid[0].change_parameters(ROBOTIC_ARM_PID_V2I_PARAMS);
    v2i_pid[0].clear_i_out();
    v2i_pid[1].change_parameters(ROBOTIC_ARM_PID_V2I_PARAMS);
    v2i_pid[1].clear_i_out();
    extend_state = 0;
    clamp_state = 0;
    slide_x_state = 0;
    should_set_arm_normal = 0;
    air_tank_interface::init();
    roboticArmThread.start(skd_thread_prio);
    palSetPad(GPIOH, POWER_PAD);
}

void RoboticArmSKD::stretch_out() {
    if (released && ((EngineerInterface::present_angle[ROBOTIC_LEFT] < ROBOTIC_ARM_PULL_BACK_ANGLE)
                    ||(EngineerInterface::present_angle[ROBOTIC_RIGHT]< ROBOTIC_ARM_PULL_BACK_ANGLE))){
        released = false;
        trigger_angle = ROBOTIC_ARM_STRETCH_OUT_ANGLE;
        target_velocity[0] = target_velocity[1] = ROBOTIC_ARM_ROTATE_VELOCITY + 250;
    }
}

void RoboticArmSKD::pull_back() {
    if (released && ((EngineerInterface::present_angle[ROBOTIC_LEFT] > ROBOTIC_ARM_STRETCH_OUT_ANGLE)
                    ||(EngineerInterface::present_angle[ROBOTIC_RIGHT]> ROBOTIC_ARM_STRETCH_OUT_ANGLE))){
        released = false;
        trigger_angle = ROBOTIC_ARM_PULL_BACK_ANGLE;
        target_velocity[0] = target_velocity[1] = - ROBOTIC_ARM_ROTATE_VELOCITY;
    }
}

/**
 * The engineer should use the function to set the air tanks
 */
void RoboticArmSKD::set_air_tank(int clamp, int slide_y, int slide_x) {
    if (clamp != -1)
        clamp_state = clamp;
    if (slide_y != -1)
        extend_state = slide_y;
    if (slide_x != -1)
        slide_x_state = slide_x;
    return;
}

//managing states, variable released should be False
void RoboticArmSKD::next_step() {
    LOG("%d %d\n", extend_state, clamp_state);
    if (!released) return;
    if (state == NORMAL) {
        state = COLLECT_BULLET;
        chThdSleepMilliseconds(1000);
        air_tank_interface::set_tank(SLIDE_Y,extend_state);
        air_tank_interface::set_tank(CLAMP,clamp_state);
        switch (slide_x_state) {
            case 0:
                air_tank_interface::set_tank(SLIDE_X_1, 0);
                air_tank_interface::set_tank(SLIDE_X_2, 0);
                break;
            case 1:
                air_tank_interface::set_tank(SLIDE_X_1, 1);
                air_tank_interface::set_tank(SLIDE_X_2, 0);
                break;
            case 2:
                air_tank_interface::set_tank(SLIDE_X_1, 1);
                air_tank_interface::set_tank(SLIDE_X_2, 1);
                break;
            default:
                break;
        }
        chThdSleepMilliseconds(1000);
        stretch_out();
    }
    else {
        switch (bullet_state) {
            case WAITING:
                bullet_state = BOX_CLAMPED;
                air_tank_interface::set_tank(SLIDE_Y,extend_state);
                air_tank_interface::set_tank(CLAMP,clamp_state);
                switch (slide_x_state) {
                    case 0:
                        air_tank_interface::set_tank(SLIDE_X_1, 0);
                        air_tank_interface::set_tank(SLIDE_X_2, 0);
                        break;
                    case 1:
                        air_tank_interface::set_tank(SLIDE_X_1, 1);
                        air_tank_interface::set_tank(SLIDE_X_2, 0);
                        break;
                    case 2:
                        air_tank_interface::set_tank(SLIDE_X_1, 1);
                        air_tank_interface::set_tank(SLIDE_X_2, 1);
                        break;
                    default:
                        break;
                }
                stretch_out();
                break;
            case BOX_CLAMPED:
                air_tank_interface::set_tank(SLIDE_Y,clamp_state);
                air_tank_interface::set_tank(SLIDE_Y,extend_state);
                pull_back();
                bullet_state = TAKING_BULLET;
                break;
            case TAKING_BULLET:
                air_tank_interface::set_tank(SLIDE_Y,clamp_state);
                air_tank_interface::set_tank(SLIDE_Y,extend_state);
                //another function here to pop the ammo bin

                bullet_state = WAITING;
                chThdSleepMilliseconds(500);
                break;
            default:
                break;
        }
    }
}
//managing states, variable released should be False
///////////////////////////////////////////
//我还没写完这个函数QAQ
void RoboticArmSKD::prev_step() {
    if (!released) return;
    if (state == NORMAL) return;
    switch (bullet_state) {
        case WAITING:
            pull_back();
            state = NORMAL;
            break;
        case BOX_CLAMPED:
            air_tank_interface::set_tank(CLAMP,clamp_state);
            bullet_state = WAITING;
            break;
        case TAKING_BULLET:
            stretch_out();
            bullet_state = BOX_CLAMPED;
            break;
    }
}

/*There are two basic components here: states changing mode when released == True and current managing mode when released == False
 *In the state managing: The states changing mechanism is like a finite state machine which specifies what to do next.
 *                       Note that human operators use functions such as next_step() and prev_step() to switch between states.
 *In the current managing: Note that the function pull_back and stretch_out will set the variable released to False,
 *                         and this tells the program that it's time to specify the current for the motors with PID.
 */
void RoboticArmSKD::update_target_current() {
    // State managing
    if (released) {
        if (state == COLLECT_BULLET) {
            if (should_set_arm_normal){
                state = NORMAL;
            }
        }
        else {//NORMAL state
            clamp_state = extend_state = slide_x_state = 0;
            air_tank_interface::set_tank(SLIDE_Y,extend_state);
            air_tank_interface::set_tank(CLAMP,clamp_state);
            air_tank_interface::set_tank(SLIDE_X_1, 0);
            air_tank_interface::set_tank(SLIDE_X_2, 0);
        }
    }
    // Current managing
    if (!released){
        if ((target_velocity[0] * (EngineerInterface::present_angle[ROBOTIC_LEFT] - trigger_angle) > 0)||
            (target_velocity[1] * (EngineerInterface::present_angle[ROBOTIC_RIGHT] - trigger_angle) > 0)){
            //already excel the trigger angle: set the target velocity to zero
            target_velocity[0] = target_velocity[1] = 0;
        }
        if ((target_velocity[0] == 0 && ABS_IN_RANGE(EngineerInterface::present_velocity[ROBOTIC_LEFT], ROBOTIC_ARM_TRIGGER_VELOCITY))||
            (target_velocity[1] == 0 && ABS_IN_RANGE(EngineerInterface::present_velocity[ROBOTIC_RIGHT], ROBOTIC_ARM_TRIGGER_VELOCITY))){
            //Can anyone explain this condition? I am quite confused...
            released = true;
            v2i_pid[0].clear_i_out();
            v2i_pid[1].clear_i_out();
        }
        if (((EngineerInterface::present_angle[ROBOTIC_LEFT] - ROBOTIC_ARM_THROW_TRIGGER) > 0 ||
            (EngineerInterface::present_angle[ROBOTIC_RIGHT] - ROBOTIC_ARM_THROW_TRIGGER) > 0) &&
            target_velocity[0] > 0 && target_velocity[1] > 0 && clamp_state == 1) {
            //Can anyone explain this condition? I am quite confused...
            clamp_state = 0;
            air_tank_interface::set_tank(CLAMP,clamp_state);
        }
        EngineerInterface::target_current[0] = (int16_t ) v2i_pid[0].calc(EngineerInterface::present_velocity[ROBOTIC_LEFT], target_velocity[0]);
        EngineerInterface::target_current[1] = (int16_t ) v2i_pid[1].calc(EngineerInterface::present_velocity[ROBOTIC_RIGHT], target_velocity[1]);
    } else{
        EngineerInterface::target_current[ROBOTIC_LEFT] = EngineerInterface::target_current[ROBOTIC_RIGHT] = 0;
    }
}
void RoboticArmSKD::RoboticArmThread::main() {
    setName("robotic_arm");
    while (!shouldTerminate()){
        update_target_current();
        EngineerInterface::data_to_can();
        sleep(TIME_MS2I(2));
    }
}