//
// Created by Kerui Zhu on 7/11/2019.
//

#include "robotic_arm_skd.h"
#include "math.h"

RoboticArmSKD::RoboticArmThread RoboticArmSKD::roboticArmThread;
RoboticArmSKD::robotic_arm_state_t RoboticArmSKD::state;
RoboticArmSKD::digital_status_t RoboticArmSKD::door_state;
RoboticArmSKD::digital_status_t RoboticArmSKD::lift_state;
RoboticArmSKD::digital_status_t RoboticArmSKD::extend_state;
bool RoboticArmSKD::released;
float RoboticArmSKD::trigger_angle;
float RoboticArmSKD::target_velocity;
PIDController RoboticArmSKD::v2i_pid;


void RoboticArmSKD::init() {
    set_clamp_action(CLAMP_RELAX);
    state = WAITING;
    released = true;
    target_velocity = 0;
    v2i_pid.change_parameters(ROBOTIC_ARM_PID_V2I_PARAMS);
    v2i_pid.clear_i_out();

    door_state = HIGH_STATUS;
    extend_state = HIGH_STATUS;
    lift_state = HIGH_STATUS;''
    change_status(door_state, DOOR_PAD);
    change_status(extend_state, EXTEND_PAD);
    change_status(lift_state, LIFT_PAD);
}

void RoboticArmSKD::set_clamp_action(clamp_status_t target_status) {
    palWritePad(GPIOH, CLAMP_PAD, target_status);
    if (target_status == CLAMP_CLAMPED) state = BOX_CLAMPED;
    else state = DOWN;
}

void RoboticArmSKD::stretch_out() {
    if (released && RoboticArmIF::present_angle < ROBOTIC_ARM_PULL_BACK_ANGLE){
        released = false;
        trigger_angle = ROBOTIC_ARM_STRETCH_OUT_ANGLE;
        target_velocity = ROBOTIC_ARM_ROTATE_VELOCITY;
        state = THROW_AWAY;
    }
}

void RoboticArmSKD::pull_back() {
    if (released && RoboticArmIF::present_angle > ROBOTIC_ARM_STRETCH_OUT_ANGLE){
        released = false;
        trigger_angle = ROBOTIC_ARM_PULL_BACK_ANGLE;
        target_velocity = - ROBOTIC_ARM_ROTATE_VELOCITY;
    }
    if (state == BOX_CLAMPED) state = TAKING_BOX;
}

void RoboticArmSKD::change_extend() {
    if (extend_state == HIGH_STATUS) extend_state = LOW_STATUS;
    else extend_state = HIGH_STATUS;
    palWritePad(GPIOH, EXTEND_PAD, extend_state);
}

void RoboticArmSKD::change_status(digital_status_t& status, uint8_t pad) {
    if (released) {
        if (status == HIGH_STATUS) status = LOW_STATUS;
        else status = HIGH_STATUS;
        palWritePad(GPIOH, pad, status);
    }
}

void RoboticArmSKD::set_status(digital_status_t& status, uint8_t pad, digital_status_t state) {
    if (released) {
        palWritePad(GPIOH, pad, state);
    }
}

void RoboticArmSKD::next_step() {
    if (!released) return;
    switch (state) {
        case WAITING:
            state = LIFT;
            set_status(lift_state, LIFT_PAD, HIGH_STATUS);
            break;
        case LIFT:
            set_clamp_action(CLAMP_CLAMPED);
            break;
        case BOX_CLAMPED:
            pull_back();
            break;
        case DOWN:
            state = WAITING;
            set_status(lift_state, LIFT_PAD, LOW_STATUS);
            break;
        default:
            break;
    }
}

void RoboticArmSKD::prev_step() {
    if (!released) return;
    switch (state) {
        case LIFT:
            set_status(lift_state, LIFT_PAD, LOW_STATUS);
            state = WAITING;
            break;
        case BOX_CLAMPED:
            set_clamp_action(CLAMP_RELAX);
            state = LIFT;
            break;
        default:
            break;
    }
}

void RoboticArmSKD::update_target_current() {
    // State managing
    switch (state){
        case TAKING_BOX:
            if (released) state = TAKING_BULLET;
            break;
        case TAKING_BULLET:
            chThdSleepMilliseconds(4000);
            stretch_out();
            break;
        case THROW_AWAY:
            if (released) set_clamp_action(CLAMP_RELAX);
        default:
            break;
    }
    // Current managing
    if (!released){
        if (target_velocity * (RoboticArmIF::present_angle - trigger_angle) > 0)
            target_velocity = 0;
        if (target_velocity == 0 && abs(RoboticArmIF::present_velocity) < ROBOTIC_ARM_TRIGGER_VELOCITY){
            released = true;
            v2i_pid.clear_i_out();
        }
        RoboticArmIF::motor_target_current = (int16_t ) v2i_pid.calc(RoboticArmIF::present_velocity, target_velocity);
    } else{
        RoboticArmIF::motor_target_current = 0;
    }
}

void RoboticArmSKD::RoboticArmThread::main() {
    setName("robotic_arm");
    init();
    while (!shouldTerminate()){
        update_target_current();
        RoboticArmIF::send_current();
        sleep(TIME_MS2I(2));
    }
}
