//
// Created by Kerui Zhu on 7/11/2019.
// Modified by Zhu Zhongbo on 2/13/2020
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
    should_set_arm_normal = 0;  //always set the state to be NORMAL
    AirTankIF::init();
    roboticArmThread.start(skd_thread_prio);
    palSetPad(GPIOH, POWER_PAD);
}

void RoboticArmSKD::stretch_out() {
    if (released && ((EngineerInterface::feedback[ROBOTIC_LEFT]->actual_angle < ROBOTIC_ARM_PULL_BACK_ANGLE)
                    ||(EngineerInterface::feedback[ROBOTIC_RIGHT]->actual_angle< ROBOTIC_ARM_PULL_BACK_ANGLE))){
        released = false;
        trigger_angle = ROBOTIC_ARM_STRETCH_OUT_ANGLE;
        target_velocity[0] = target_velocity[1] = ROBOTIC_ARM_ROTATE_VELOCITY + 250;
    }
}

void RoboticArmSKD::pull_back() {
    if (released && ((EngineerInterface::feedback[ROBOTIC_LEFT]->actual_angle  > ROBOTIC_ARM_STRETCH_OUT_ANGLE)
                    ||(EngineerInterface::feedback[ROBOTIC_RIGHT]->actual_angle > ROBOTIC_ARM_STRETCH_OUT_ANGLE))){
        released = false;
        trigger_angle = ROBOTIC_ARM_PULL_BACK_ANGLE;
        target_velocity[0] = target_velocity[1] = - ROBOTIC_ARM_ROTATE_VELOCITY;
    }
}

/**
 * The engineer should use the function to set the air tanks
 * All of the changes made to the three states of robotic arm (air tank part) should be made through this function
 */
void RoboticArmSKD::set_air_tank(int clamp, int slide_y, int slide_x) {
    if (clamp != -1)
        clamp_state = clamp;
    if (slide_y != -1)
        extend_state = slide_y;
    if (slide_x != -1)
        slide_x_state = slide_x;
}

//managing states, variable released should be False
//中文强调：每次使用这个next step之前都需要在logic模块里面更新气缸目标状态，并用set_air_tank来赋值（不要直接改变量）
//需要等待多久要测试，例如气缸反应速度之类的
void RoboticArmSKD::next_step() {
    LOG("%d %d\n", extend_state, clamp_state);
    if (!released) return;
    if (state == NORMAL) {
        state = COLLECT_BULLET;
    }
    else {
        switch (bullet_state) {
            case WAITING://在起始位置，夹子松
                bullet_state = BOX_CLAMPED;
                chThdSleepMilliseconds(1000);
                AirTankIF::set_tank(SLIDE_Y,extend_state);
                AirTankIF::set_tank(CLAMP,clamp_state);
                switch (slide_x_state) {
                    case 0:
                        AirTankIF::set_tank(SLIDE_X_1, 0);
                        AirTankIF::set_tank(SLIDE_X_2, 0);
                        break;
                    case 1:
                        AirTankIF::set_tank(SLIDE_X_1, 1);
                        AirTankIF::set_tank(SLIDE_X_2, 0);
                        break;
                    case 2:
                        AirTankIF::set_tank(SLIDE_X_1, 1);
                        AirTankIF::set_tank(SLIDE_X_2, 1);
                        break;
                    default:
                        break;
                }
                chThdSleepMilliseconds(1000);
                stretch_out();
                break;
            case BOX_CLAMPED://已旋转，平移，尚未夹紧
                AirTankIF::set_tank(CLAMP,clamp_state);
                AirTankIF::set_tank(SLIDE_Y,extend_state);
                chThdSleepMilliseconds(1000);
                pull_back();
                bullet_state = TAKING_BULLET;
                break;
            case TAKING_BULLET://已经夹紧并旋转平移回复到初始状态
                AirTankIF::set_tank(CLAMP,clamp_state);
                AirTankIF::set_tank(SLIDE_Y,extend_state);
                //another function here to pop the ammo bin
                EngineerInterface::pop_ammo_bin();
                bullet_state = WAITING;
                break;
            default:
                break;
        }
    }
}

//managing states, variable released should be False
//this function will be used for debugging the machine: return to previous states
//this function will also make use of set_air_tank()
//这里所谓的前一步是一个周期（静止-伸出-收回）的前一步，不会对slide_x_state进行修改
void RoboticArmSKD::prev_step() {
    if (!released) return;
    if (state == NORMAL) return;
    switch (bullet_state) {
        case WAITING://此时夹子松开正处于初始位置
            state = NORMAL;//回归NORMAL
            set_air_tank(0,0,-1);
            AirTankIF::set_tank(CLAMP,clamp_state);
            AirTankIF::set_tank(SLIDE_Y,extend_state);
            break;
        case BOX_CLAMPED://此时夹子还是松的，但是到达了指定的位置
            set_air_tank(-1,0,-1);
            AirTankIF::set_tank(CLAMP,clamp_state);
            AirTankIF::set_tank(SLIDE_Y,extend_state);
            chThdSleepMilliseconds(1000);
            pull_back();
            bullet_state = WAITING;
            break;
        case TAKING_BULLET://此状态下夹子夹紧了，并且已经回复，倒退时需要把y轴伸出去
            set_air_tank(-1,1,-1);
            stretch_out();//先伸出，后松夹子
            AirTankIF::set_tank(SLIDE_Y,extend_state);
            chThdSleepMilliseconds(1000);
            AirTankIF::set_tank(CLAMP,clamp_state);
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
                //judge whether we should set the state of robotic arm back to NORMAL state(not collecting bullet)
                state = NORMAL;
            }
        }
        else {//NORMAL state
            set_air_tank(0,0,0);
            AirTankIF::set_tank(SLIDE_Y,extend_state);
            AirTankIF::set_tank(CLAMP,clamp_state);
            AirTankIF::set_tank(SLIDE_X_1, 0);
            AirTankIF::set_tank(SLIDE_X_2, 0);
            target_velocity[0] = target_velocity[1] = 0;
            EngineerInterface::target_current[ROBOTIC_LEFT] = EngineerInterface::target_current[ROBOTIC_RIGHT] = 0;
        }
    }
    // Current managing(during the process of pull back or stretch out)
    if (!released){
        if ((target_velocity[0] * (EngineerInterface::feedback[ROBOTIC_LEFT]->actual_angle  - trigger_angle) > 0)||
            (target_velocity[1] * (EngineerInterface::feedback[ROBOTIC_RIGHT]->actual_angle  - trigger_angle) > 0)){
            //already excel the trigger angle: set the target velocity to zero
            target_velocity[0] = target_velocity[1] = 0;
        }
        if ((target_velocity[0] == 0 && ABS_IN_RANGE(EngineerInterface::feedback[ROBOTIC_LEFT]->actual_velocity, ROBOTIC_ARM_TRIGGER_VELOCITY))||
            (target_velocity[1] == 0 && ABS_IN_RANGE(EngineerInterface::feedback[ROBOTIC_RIGHT]->actual_velocity, ROBOTIC_ARM_TRIGGER_VELOCITY))){
            //already close to the target current
            released = true;
            v2i_pid[0].clear_i_out();
            v2i_pid[1].clear_i_out();
        }
//        if (((EngineerInterface::present_angle[ROBOTIC_LEFT] - ROBOTIC_ARM_THROW_TRIGGER) > 0 ||
//            (EngineerInterface::present_angle[ROBOTIC_RIGHT] - ROBOTIC_ARM_THROW_TRIGGER) > 0) &&
//            target_velocity[0] > 0 && target_velocity[1] > 0 && clamp_state == 1) {
//            //
//            clamp_state = 0;
//            AirTankIF::set_tank(CLAMP,clamp_state);
//        }
        *(EngineerInterface::target_current[ROBOTIC_LEFT]) = v2i_pid[0].calc(EngineerInterface::feedback[ROBOTIC_LEFT]->actual_velocity, target_velocity[0]);
        *(EngineerInterface::target_current[ROBOTIC_RIGHT]) = v2i_pid[1].calc(EngineerInterface::feedback[ROBOTIC_RIGHT]->actual_velocity, target_velocity[1]);
    }
}

void RoboticArmSKD::RoboticArmThread::main() {
    setName("robotic_arm");
    while (!shouldTerminate()){
        update_target_current();
        sleep(TIME_MS2I(2));
    }
}