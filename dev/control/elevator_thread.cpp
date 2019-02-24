//
// Created by liuzikai on 2019-02-24.
//

#include "elevator_thread.h"

ElevatorThread::status_t ElevatorThread::get_status() {
    return status;
}

bool ElevatorThread::start_up_actions(tprio_t prio) {
    if (status != STOP) return false;
    status = UPWARD;
    chibios_rt::BaseStaticThread<ELEVATOR_THREAD_WORKING_AREA_SIZE>::start(prio);
    return true;
}

bool ElevatorThread::start_down_actions(tprio_t prio) {
    if (status != STOP) return false;
    status = DOWNWARD;
    chibios_rt::BaseStaticThread<ELEVATOR_THREAD_WORKING_AREA_SIZE>::start(prio);
    return true;
}

void ElevatorThread::emergency_stop() {
    status = STOP;
    exit(0xFF);
}

void ElevatorThread::main() {
    setName("elevator");

    systime_t step_start_time;

    if (status == STOP) {
        exit(-1);
    } else if (status == UPWARD) {

        // Step 1. Lower all wheels
        ElevatorInterface::apply_front_position(-stage_height);
        ElevatorInterface::apply_rear_position(-stage_height);
        while (ElevatorInterface::elevator_wheels[0].is_in_action() ||
               ElevatorInterface::elevator_wheels[1].is_in_action() ||
               ElevatorInterface::elevator_wheels[2].is_in_action() ||
               ElevatorInterface::elevator_wheels[3].is_in_action()) {
            sleep(TIME_MS2I(elevator_check_interval));
        }

        // Step 2. Move forward to make the front assistant wheels to get onto the stage
        step_start_time = chVTGetSystemTime();
        while (TIME_I2MS(chVTGetSystemTime() - step_start_time) >= 2000) {

            // Pack the actual velocity into an array
            float measured_velocity[4];
            for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                measured_velocity[i] = ChassisInterface::motor[i].actual_angular_velocity;
            }

            // Perform calculation
            ChassisController::calc(measured_velocity, 0, 10, 0);

            // Pass the target current to interface
            for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                ChassisInterface::motor[i].target_current = (int) ChassisController::motor[i].target_current;
            }

            ChassisInterface::send_chassis_currents();

            sleep(TIME_MS2I(chassis_action_interval));
        }

        // Step 3. Lift the front wheels
        ElevatorInterface::apply_front_position(0);
        while (ElevatorInterface::elevator_wheels[0].is_in_action() ||
               ElevatorInterface::elevator_wheels[1].is_in_action()) {
            sleep(TIME_MS2I(elevator_check_interval));
        }

        // Step 4. Move forward to make the front wheels and rear assistant wheel to be on the stage
        step_start_time = chVTGetSystemTime();
        while (TIME_I2MS(chVTGetSystemTime() - step_start_time) >= 1000) {

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

        // Step 5. Lift the rear wheels
        ElevatorInterface::apply_rear_position(0);
        while (ElevatorInterface::elevator_wheels[2].is_in_action() ||
               ElevatorInterface::elevator_wheels[3].is_in_action()) {
            sleep(TIME_MS2I(elevator_check_interval));
        }

        // Step 4. Move forward to make the rear wheels and be on the stage
        step_start_time = chVTGetSystemTime();
        while (TIME_I2MS(chVTGetSystemTime() - step_start_time) >= 2000) {

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
        exit(0);
    }
}

