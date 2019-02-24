//
// Created by liuzikai on 2019-02-24.
//

#include "robotic_arm_thread.h"

RoboticArmThread::status_t RoboticArmThread::get_status() {
    return status;
}

bool RoboticArmThread::start_actions(tprio_t prio) {
    if (status != STOP) return false;
    if (!ABS_IN_RANGE(RoboticArm::get_motor_actual_angle() - rotation_motor_inside_angle_raw, 100))
        return false;
    status = ACTIONING;
    chibios_rt::BaseStaticThread<ROBOTIC_ARM_THREAD_WORKING_AREA_SIZE>::start(prio);
    return true;
}

void RoboticArmThread::emergency_stop() {
    status = STOP;
    exit(0xFF);
}

void RoboticArmThread::main() {
    setName("robotic_arm");

    if (!ABS_IN_RANGE(RoboticArm::get_motor_actual_angle() - rotation_motor_inside_angle_raw, 100)) {
        exit(1);
    }

    // TODO: determine the compare sign here
    while (RoboticArm::get_motor_actual_angle() < rotation_motor_outward_trigger_angle_raw) {

        // Pack the actual velocity into an array
        float measured_velocity[4];
        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
            measured_velocity[i] = ChassisInterface::motor[i].actual_angular_velocity;
        }

        // Perform calculation
        ChassisController::calc(measured_velocity, 0, 100, 0);

        // Pass the target current to interface
        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
            ChassisInterface::motor[i].target_current = (int) ChassisController::motor[i].target_current;
        }

        ChassisInterface::send_chassis_currents();

        sleep(TIME_MS2I(chassis_action_interval));
    }

    // Complete
    status = STOP;
    exit(0);
}