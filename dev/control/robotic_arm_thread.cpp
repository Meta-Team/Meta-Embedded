//
// Created by liuzikai on 2019-02-24.
//

#include "robotic_arm_thread.h"
#include "buzzer.h"
#include "led.h"

RoboticArmThread::status_t RoboticArmThread::get_status() {
    return status;
}

bool RoboticArmThread::start_actions(tprio_t prio) {
    if (status != STOP) return false;
    if (!ABS_IN_RANGE(RoboticArm::get_motor_actual_angle() - motor_inside_target_angle, 100))
        return false;
    status = ACTIONING;
    chibios_rt::BaseStaticThread<ROBOTIC_ARM_THREAD_WORKING_AREA_SIZE>::start(prio);
    return true;
}

void RoboticArmThread::emergency_stop() {
    status = STOP;
    exit(0xFF); // FIXME: this won't work since it terminate the call thread
}

void RoboticArmThread::main() {

    setName("robotic_arm");

    if (!ABS_IN_RANGE(RoboticArm::get_motor_actual_angle() - motor_inside_target_angle, 5)) {
        exit(1);
    }

    // Step 1. Rotate to motor_outward_boundary_angle

    RoboticArm::set_motor_target_current(-2000);
    while (RoboticArm::get_motor_actual_angle() > -65) {
        RoboticArm::send_motor_target_current();
        sleep(TIME_MS2I(motor_action_interval));
    }

    // TODO: complete comments
    // Step 1.5. Rotate to motor_outward_boundary_angle

    RoboticArm::set_motor_target_current(0);
    while (RoboticArm::get_motor_actual_angle() > -95) {
        RoboticArm::send_motor_target_current();
        sleep(TIME_MS2I(motor_action_interval));
    }

    // Step 2. Wait for motor to reach motor_outward_boundary_angle

    RoboticArm::set_motor_target_current(800);
    while (RoboticArm::get_motor_actual_angle() > -150) {
        RoboticArm::send_motor_target_current();
        sleep(TIME_MS2I(motor_action_interval));
    }

    RoboticArm::set_motor_target_current(0);
    RoboticArm::send_motor_target_current();
    sleep(TIME_MS2I(2000));

    // Step 3. Clamp
    RoboticArm::clamp_action(RoboticArm::CLAMP_CLAMPED);
    sleep(TIME_MS2I(2000));

    // Step 4. Rotate to motor_inward_boundary_angle
    RoboticArm::set_motor_target_current(10000);
    while (RoboticArm::get_motor_actual_angle() < -67) {
        RoboticArm::send_motor_target_current();
        sleep(TIME_MS2I(motor_action_interval));
    }

    RoboticArm::set_motor_target_current(0);
    while (RoboticArm::get_motor_actual_angle() < -40) {
        RoboticArm::send_motor_target_current();
        sleep(TIME_MS2I(motor_action_interval));
    }

//    // Step 5. Wait for motor to reach motor_inside_target_angle
//    RoboticArm::set_motor_target_current(-1200);
//    while (RoboticArm::get_motor_actual_angle() < -5) {
//        RoboticArm::send_motor_target_current();
//        sleep(TIME_MS2I(motor_action_interval));
//    }

    // Step
    RoboticArm::set_motor_target_current(0);
    RoboticArm::send_motor_target_current();
    sleep(TIME_MS2I(2000));

    // Step
    RoboticArm::set_motor_target_current(-8000);
    while (RoboticArm::get_motor_actual_angle() > -65) {
        RoboticArm::send_motor_target_current();
        sleep(TIME_MS2I(motor_action_interval));
    }

    RoboticArm::clamp_action(RoboticArm::CLAMP_RELAX);

    // Complete

    RoboticArm::set_motor_target_current(0);
    RoboticArm::send_motor_target_current();

    status = STOP;
    exit(0);
}