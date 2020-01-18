//
// Created by Kerui Zhu on 7/11/2019.
//

#include "robotic_arm_skd.h"

RoboticArmSKD::RoboticArmThread RoboticArmSKD::roboticArmThread;
RoboticArmSKD::robotic_arm_state_t RoboticArmSKD::state;
RoboticArmSKD::bullet_state_t RoboticArmSKD::bullet_state;
RoboticArmSKD::digital_status_t RoboticArmSKD::door_state;
RoboticArmSKD::digital_status_t RoboticArmSKD::lift_state;
RoboticArmSKD::digital_status_t RoboticArmSKD::extend_state;
RoboticArmSKD::digital_status_t RoboticArmSKD::clamp_state;
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

//managing states, variable released should be False
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
//managing states, variable released should be False
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
            switch (bullet_state) {
                case WAITING:
                    set_digital_status(clamp_state, CLAMP_PAD, LOW_STATUS);
                    break;
                case TAKING_BULLET:
                    chThdSleepMilliseconds(500);
                    stretch_out();
                    bullet_state = WAITING;
                    //In the function next_step, the status will end up in TAKING BULLET,
                    //and this update_target_current function will reset the status of robotic arm back to WAITING.
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
        if ((target_velocity[0] * (EngineerInterface::present_angle[ROBOTIC_LEFT] - trigger_angle) > 0)||
            (target_velocity[1] * (EngineerInterface::present_angle[ROBOTIC_RIGHT] - trigger_angle) > 0)){
            target_velocity[0] = target_velocity[1] = 0;
        }
        if ((target_velocity[0] == 0 && ABS_IN_RANGE(EngineerInterface::present_velocity[ROBOTIC_LEFT], ROBOTIC_ARM_TRIGGER_VELOCITY))||
            (target_velocity[1] == 0 && ABS_IN_RANGE(EngineerInterface::present_velocity[ROBOTIC_RIGHT], ROBOTIC_ARM_TRIGGER_VELOCITY))){
            released = true;
            v2i_pid[0].clear_i_out();
            v2i_pid[1].clear_i_out();
        }
        if (((EngineerInterface::present_angle[ROBOTIC_LEFT] - ROBOTIC_ARM_THROW_TRIGGER) > 0 ||
            (EngineerInterface::present_angle[ROBOTIC_RIGHT] - ROBOTIC_ARM_THROW_TRIGGER) > 0) &&
            target_velocity[0] > 0 && target_velocity[1] > 0 && clamp_state == HIGH_STATUS) {

            clamp_state = LOW_STATUS;
            palClearPad(GPIOE, CLAMP_PAD);
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