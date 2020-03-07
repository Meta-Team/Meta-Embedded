//
// Created by Kerui Zhu on 2/17/2020.
//

#include "new_robotic_arm_skd.h"

NewRoboticArmSkd::RoboticArmThread NewRoboticArmSkd::roboticArmThread;
NewRoboticArmSkd::RA_motor_state_t NewRoboticArmSkd::motorState;
NewRoboticArmSkd::RA_motor_instruction NewRoboticArmSkd::targetState;
int NewRoboticArmSkd::x_slide_state;
float NewRoboticArmSkd::target_velocity;
float NewRoboticArmSkd::trigger_angle;
PIDController NewRoboticArmSkd::v2i_pid[2];

void NewRoboticArmSkd::start(tprio_t skd_thread_prio) {
    target_velocity = 0;
    v2i_pid[0].change_parameters(ROBOTIC_ARM_PID_V2I_PARAMS);
    v2i_pid[0].clear_i_out();
    v2i_pid[1].change_parameters(ROBOTIC_ARM_PID_V2I_PARAMS);
    v2i_pid[1].clear_i_out();
    //I think it's necessary to make them have a initial value
    motorState = RETRIEVED;
    targetState = RETRIEVE;
    roboticArmThread.start(skd_thread_prio);
    palSetPad(GPIOH, POWER_PAD);
}

NewRoboticArmSkd::RA_motor_state_t NewRoboticArmSkd::get_motor_state() {
    return motorState;
}

void NewRoboticArmSkd::set_motor_instruction(RA_motor_instruction command) {
    if (targetState == command) return;

    targetState = command;
    v2i_pid[0].clear_i_out();
    v2i_pid[1].clear_i_out();
    if (targetState == STRETCH){
        target_velocity = ROBOTIC_ARM_ROTATE_VELOCITY + 250;
        motorState = STRETCHING;
        trigger_angle = ROBOTIC_ARM_STRETCH_OUT_ANGLE;
    } else{
        target_velocity = -ROBOTIC_ARM_ROTATE_VELOCITY;
        motorState = RETRIEVING;
        trigger_angle = ROBOTIC_ARM_PULL_BACK_ANGLE;
    }
}

bool NewRoboticArmSkd::is_extended() {
    return AirTankIF::get_status(SLIDE_Y);
}

void NewRoboticArmSkd::set_extension(bool extend) {
    AirTankIF::set_tank(SLIDE_Y, extend);
}

bool NewRoboticArmSkd::is_clamped() {
    return AirTankIF::get_status(CLAMP);
}

void NewRoboticArmSkd::set_clamp(bool clamp) {
    AirTankIF::set_tank(CLAMP, clamp);
}

int NewRoboticArmSkd::get_x_slide_state() {
    return x_slide_state;
}

void NewRoboticArmSkd::set_x_slide_state(int state) {
    if (state < 0 || state > 2)
        return;
    x_slide_state = state;
    switch (x_slide_state) {
        case 0:
            AirTankIF::set_tank(SLIDE_X_1, false);
            AirTankIF::set_tank(SLIDE_X_2, false);
            break;
        case 1:
            AirTankIF::set_tank(SLIDE_X_1, true);
            AirTankIF::set_tank(SLIDE_X_2, false);
            break;
        case 2:
            AirTankIF::set_tank(SLIDE_X_1, true);
            AirTankIF::set_tank(SLIDE_X_2, true);
            break;
        default:
            break;
    }
}

void NewRoboticArmSkd::update() {
    switch (motorState){
        case RETRIEVED:
            if (targetState == RETRIEVE){
                RoboticArmIF::motor_target_current = 0;
            } else{
                motorState = STRETCHING;
                calculate_current();
            }
            break;
        case STRETCHED:
            if (targetState == STRETCH){
                RoboticArmIF::motor_target_current = 0;
            } else{
                motorState = RETRIEVING;
                calculate_current();
            }
            break;
//It seems that the following code is not complete, please modify them.
        case RETRIEVING:
//            if (RoboticArmIF::present_velocity * (RoboticArmIF::present_angle - ))
//            break;
        case STRETCHING:
            break;
    }
}

void NewRoboticArmSkd::calculate_current() {

}

void NewRoboticArmSkd::RoboticArmThread::main() {
    setName("new_robotic_arm");
    while (!shouldTerminate()){
        update();
        RoboticArmIF::send_current();
        sleep(TIME_MS2I(2));
    }
}
