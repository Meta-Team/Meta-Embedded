//
// Created by Kerui Zhu on 7/11/2019.
//

#include "robotic_arm_skd.h"
#include "math.h"

RoboticArmSKD::RoboticArmThread RoboticArmSKD::roboticArmThread;
RoboticArmSKD::robotic_arm_state_t RoboticArmSKD::state;
RoboticArmSKD::bullet_state_t RoboticArmSKD::bullet_state;
RoboticArmSKD::digital_status_t RoboticArmSKD::door_state;
RoboticArmSKD::digital_status_t RoboticArmSKD::lift_state;
RoboticArmSKD::digital_status_t RoboticArmSKD::extend_state;
RoboticArmSKD::digital_status_t RoboticArmSKD::clamp_state;
bool RoboticArmSKD::released;
float RoboticArmSKD::trigger_angle;
float RoboticArmSKD::target_velocity;
PIDController RoboticArmSKD::v2i_pid;


void RoboticArmSKD::start(tprio_t skd_thread_prio) {

    state = NORMAL;
    bullet_state = WAITING;
    released = true;
    target_velocity = 0;
    v2i_pid.change_parameters(ROBOTIC_ARM_PID_V2I_PARAMS);
    v2i_pid.clear_i_out();

    lift_state = LOW_STATUS;
    door_state = LOW_STATUS;
    extend_state = LOW_STATUS;
    clamp_state = LOW_STATUS;
    palClearPad(GPIOE, LIFT_PAD);
    palClearPad(GPIOE, DOOR_PAD);
    palClearPad(GPIOE, CLAMP_PAD);
    palClearPad(GPIOE, EXTEND_PAD);
    roboticArmThread.start(skd_thread_prio);
    palSetPad(GPIOH, POWER_PAD);
}

void RoboticArmSKD::stretch_out() {
    if (released && RoboticArmIF::present_angle < ROBOTIC_ARM_PULL_BACK_ANGLE){
        released = false;
        trigger_angle = ROBOTIC_ARM_STRETCH_OUT_ANGLE;
        target_velocity = ROBOTIC_ARM_ROTATE_VELOCITY + 250;
    }
}

void RoboticArmSKD::pull_back() {
    if (released && RoboticArmIF::present_angle > ROBOTIC_ARM_STRETCH_OUT_ANGLE){
        released = false;
        trigger_angle = ROBOTIC_ARM_PULL_BACK_ANGLE;
        target_velocity = - ROBOTIC_ARM_ROTATE_VELOCITY;
    }
}

void RoboticArmSKD::change_extend() {
    change_digital_status(extend_state, EXTEND_PAD);
    LOG("Extend Changed! Set %d to %d", EXTEND_PAD, extend_state);
}

void RoboticArmSKD::change_door() {
    if (door_state == LOW_STATUS) {
        chThdSleepMilliseconds(1000);
        change_digital_status(door_state, DOOR_PAD);
    }
    else {
        change_digital_status(door_state, DOOR_PAD);
    }
}

void RoboticArmSKD::change_digital_status(digital_status_t& status, uint8_t pad) {
    if (released) {
        if (status == HIGH_STATUS) status = LOW_STATUS;
        else status = HIGH_STATUS;
        palWritePad(GPIOE, pad, status);
    }
}

void RoboticArmSKD::set_digital_status(digital_status_t& status, uint8_t pad, digital_status_t d_state) {
    if (released && status != d_state) {
        status = d_state;
        LOG("set %d to %d\n",pad, d_state);
        palWritePad(GPIOE, pad, d_state);
    }
}

void RoboticArmSKD::next_step() {

    LOG("%d %d %d %d\n", extend_state, lift_state, door_state, clamp_state);
    if (!released) return;
    if (state == NORMAL) {
        state = COLLECT_BULLET;
        chThdSleepMilliseconds(1000);
        set_digital_status(extend_state, EXTEND_PAD, LOW_STATUS);
        set_digital_status(lift_state, LIFT_PAD, HIGH_STATUS);
        set_digital_status(clamp_state, CLAMP_PAD, LOW_STATUS);
        chThdSleepMilliseconds(1000);
        stretch_out();
    }
    else {
        switch (bullet_state) {
            case WAITING:
                bullet_state = BOX_CLAMPED;
                set_digital_status(clamp_state, CLAMP_PAD, HIGH_STATUS);
                break;
            case BOX_CLAMPED:
                set_digital_status(extend_state, EXTEND_PAD, LOW_STATUS);
                pull_back();
                bullet_state = TAKING_BULLET;
                break;
            default:
                break;
        }
    }
}

void RoboticArmSKD::prev_step() {
    if (!released) return;
    if (state == NORMAL) return;
    switch (bullet_state) {
        case WAITING:
            pull_back();
            state = NORMAL;
            break;
        case BOX_CLAMPED:
            set_digital_status(clamp_state, CLAMP_PAD, LOW_STATUS);
            bullet_state = WAITING;
            break;
        case TAKING_BULLET:
            stretch_out();
            bullet_state = BOX_CLAMPED;
            break;
    }
}

void RoboticArmSKD::update_target_current() {
    // State managing
    if (released) {
        if (state == COLLECT_BULLET) {
            switch (bullet_state) {
                case WAITING:
                    set_digital_status(clamp_state, CLAMP_PAD, LOW_STATUS);
                    break;
                case TAKING_BULLET:
                    chThdSleepMilliseconds(500);
                    stretch_out();
                    bullet_state = WAITING;
                    break;
                default:
                    break;
            }
        }
        else {
            set_digital_status(extend_state, EXTEND_PAD, LOW_STATUS);
            set_digital_status(lift_state, LIFT_PAD, LOW_STATUS);
            set_digital_status(clamp_state, CLAMP_PAD, LOW_STATUS);
        }
    }
    // Current managing
    if (!released){
        if (target_velocity * (RoboticArmIF::present_angle - trigger_angle) > 0)
            target_velocity = 0;
        if (target_velocity == 0 && ABS_IN_RANGE(RoboticArmIF::present_velocity, ROBOTIC_ARM_TRIGGER_VELOCITY)){
            released = true;
            v2i_pid.clear_i_out();
        }
        if ((RoboticArmIF::present_angle - ROBOTIC_ARM_THROW_TRIGGER) > 0 && target_velocity > 0 && clamp_state == HIGH_STATUS) {
            clamp_state = LOW_STATUS;
            palClearPad(GPIOE, CLAMP_PAD);
        }
        RoboticArmIF::motor_target_current = (int16_t ) v2i_pid.calc(RoboticArmIF::present_velocity, target_velocity);

    } else{
        RoboticArmIF::motor_target_current = 0;
    }
}
void RoboticArmSKD::RoboticArmThread::main() {
    setName("robotic_arm");
    while (!shouldTerminate()){
        update_target_current();
        RoboticArmIF::send_current();
        sleep(TIME_MS2I(2));
    }
}