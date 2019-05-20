//
// Created by liuzikai on 2019-02-24.
//

#include "state_machine_bullet_fetch.h"
#include "buzzer.h"
#include "led.h"

BulletFetchStateMachine::action_t BulletFetchStateMachine::get_current_action() {
    return action_;
}

bool BulletFetchStateMachine::start_initial_outward(tprio_t prio) {
    if (action_ != STOP) return false;
    action_ = INITIAL_OUTWARD;
    chibios_rt::BaseStaticThread<BULLET_FETCH_STATE_MACHINE_WORKING_AREA_SIZE>::start(prio);
    return true;
}

bool BulletFetchStateMachine::start_one_fetch(tprio_t prio) {
    if (action_ != STOP) return false;
    action_ = FETCH_ONCE;
    chibios_rt::BaseStaticThread<BULLET_FETCH_STATE_MACHINE_WORKING_AREA_SIZE>::start(prio);
    return true;
}

bool BulletFetchStateMachine::start_final_inward(tprio_t prio) {
    if (action_ != STOP) return false;
    action_ = FINAL_INWARD;
    chibios_rt::BaseStaticThread<BULLET_FETCH_STATE_MACHINE_WORKING_AREA_SIZE>::start(prio);
    return true;
}

void BulletFetchStateMachine::main() {

    setName("robotic_arm");

    if (action_ == STOP) {
        // Do nothing
    } else if (action_ == INITIAL_OUTWARD) {

        // Step 1. Rotate outwards
        RoboticArm::set_motor_target_current(-2500);
        while (RoboticArm::get_motor_actual_angle() > -65) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }

        // Step 2. Rotate outwards with inertia
        RoboticArm::set_motor_target_current(0);
        while (RoboticArm::get_motor_actual_angle() > -95) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }

        // Step 3. Rotate outwards with deceleration
        RoboticArm::set_motor_target_current(1200);
        while (RoboticArm::get_motor_actual_angle() > -150) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }

        // Wait for 500ms, finish
        RoboticArm::set_motor_target_current(0);
        RoboticArm::send_motor_target_current();
        sleep(TIME_MS2I(500));

        action_ = STOP;

    } else if (action_ == FETCH_ONCE) {

        // Step 1. Drawer goes outwards
        palSetPad(GPIOH, GPIOH_POWER4_CTRL);
        sleep(TIME_MS2I(2000));

        // Step 2. Clamp
        RoboticArm::clamp_action(RoboticArm::CLAMP_CLAMPED);
        sleep(TIME_MS2I(2000));

        // Step 3. Drawer goes inwards
        palClearPad(GPIOH, GPIOH_POWER4_CTRL);
        sleep(TIME_MS2I(2000));

        // Step 4.1. Rotate inwards
        RoboticArm::set_motor_target_current(10000);
        while (RoboticArm::get_motor_actual_angle() < -67) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }

        // Step 4.2. Rotate inwards with inertia
        RoboticArm::set_motor_target_current(0);
        while (RoboticArm::get_motor_actual_angle() < -40) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }

        /*// Step 2.3. Rotate inwards with deceleration
        RoboticArm::set_motor_target_current(-1200);
        while (RoboticArm::get_motor_actual_angle() < -5) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }*/

        // Step 3. Wait for bullets to come out
        RoboticArm::set_motor_target_current(0);
        RoboticArm::send_motor_target_current();
        sleep(TIME_MS2I(2000));

        // Step 4. Throw the box outward
        RoboticArm::set_motor_target_current(-8000);
        while (RoboticArm::get_motor_actual_angle() > -65) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }
        // Relax the clamp
        RoboticArm::clamp_action(RoboticArm::CLAMP_RELAX);

        // Wait for 500ms, finish
        RoboticArm::set_motor_target_current(0);
        RoboticArm::send_motor_target_current();

        action_ = STOP;

    } else if (action_ == FINAL_INWARD) {

        // Step 1.1. Rotate inwards
        RoboticArm::set_motor_target_current(3000);
        while (RoboticArm::get_motor_actual_angle() < -67) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }

        // Step 1.2. Rotate inwards with inertia
        RoboticArm::set_motor_target_current(0);
        while (RoboticArm::get_motor_actual_angle() < -40) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }

        // Step 1.3. Rotate inwards with deceleration
        RoboticArm::set_motor_target_current(-1500);
        while (RoboticArm::get_motor_actual_angle() < -5) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }

        // Wait for 500ms, finish
        RoboticArm::set_motor_target_current(0);
        RoboticArm::send_motor_target_current();

        action_ = STOP;

    }

    exit(0);
}

bool BulletFetchStateMachine::is_outward() {
    return (RoboticArm::get_motor_actual_angle() < -150);
}