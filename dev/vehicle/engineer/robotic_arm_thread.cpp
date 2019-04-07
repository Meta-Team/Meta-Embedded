//
// Created by liuzikai on 2019-02-24.
//

#include "robotic_arm_thread.h"
#include "buzzer.h"
#include "led.h"

RoboticArmThread::status_t RoboticArmThread::get_status() {
    return status_;
}

bool RoboticArmThread::start_initial_outward(tprio_t prio) {
    if (status_ != STOP) return false;
    status_ = INITIAL_OUTWARD;
    chibios_rt::BaseStaticThread<ROBOTIC_ARM_THREAD_WORKING_AREA_SIZE>::start(prio);
    return true;
}

bool RoboticArmThread::start_one_fetch(tprio_t prio) {
    if (status_ != STOP) return false;
    status_ = FETCH_ONCE;
    chibios_rt::BaseStaticThread<ROBOTIC_ARM_THREAD_WORKING_AREA_SIZE>::start(prio);
    return true;
}

bool RoboticArmThread::start_final_inward(tprio_t prio) {
    if (status_ != STOP) return false;
    status_ = FINAL_INWARD;
    chibios_rt::BaseStaticThread<ROBOTIC_ARM_THREAD_WORKING_AREA_SIZE>::start(prio);
    return true;
}

void RoboticArmThread::emergency_stop() {
    status_ = STOP;
    exit(0xFF); // FIXME: this won't work since it terminate the call thread
}

void RoboticArmThread::main() {

    setName("robotic_arm");

    if (status_ == STOP) {
        // Do nothing
    } else if (status_ == INITIAL_OUTWARD) {

        // Step 1. Rotate outwards
        RoboticArm::set_motor_target_current(-2000);
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
        RoboticArm::set_motor_target_current(800);
        while (RoboticArm::get_motor_actual_angle() > -150) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }

        // Wait for 500ms, finish
        RoboticArm::set_motor_target_current(0);
        RoboticArm::send_motor_target_current();
        sleep(TIME_MS2I(500));

        status_ = STOP;

    } else if (status_ == FETCH_ONCE) {

        // Step 1. Clamp
        RoboticArm::clamp_action(RoboticArm::CLAMP_CLAMPED);
        sleep(TIME_MS2I(2000));

        // Step 2.1. Rotate inwards
        RoboticArm::set_motor_target_current(10000);
        while (RoboticArm::get_motor_actual_angle() < -67) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }

        // Step 2.2. Rotate inwards with inertia
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

        status_ = STOP;

    } else if (status_ == FINAL_INWARD) {

        // Step 1.1. Rotate inwards
        RoboticArm::set_motor_target_current(7000);
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
        RoboticArm::set_motor_target_current(-1200);
        while (RoboticArm::get_motor_actual_angle() < -5) {
            RoboticArm::send_motor_target_current();
            sleep(TIME_MS2I(motor_action_interval));
        }

        // Wait for 500ms, finish
        RoboticArm::set_motor_target_current(0);
        RoboticArm::send_motor_target_current();

        status_ = STOP;

    }

    exit(0);
}

bool RoboticArmThread::is_outward() {
    return (RoboticArm::get_motor_actual_angle() < -150);
}