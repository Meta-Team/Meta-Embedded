//
// Created by liuzikai on 2019-02-24.
//

#include "elevator_thread.h"
#include "buzzer.h"

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
            ElevatorInterface::apply_front_position(-stage_height);
            ElevatorInterface::apply_rear_position(-stage_height);
            sleep(TIME_MS2I(elevator_check_interval));
        }

        sleep(TIME_MS2I(100));//

        // Step 2. Move forward to make the front assistant wheels to get onto the stage
        chassis_target_vx = -500;
        sleep(TIME_MS2I(500));

        chassis_target_vx = 0;
        sleep(TIME_MS2I(100));//

        // Step 3. Lift the front wheels
        ElevatorInterface::apply_front_position(0);
        while (ElevatorInterface::elevator_wheels[0].is_in_action() ||
               ElevatorInterface::elevator_wheels[1].is_in_action()) {
            ElevatorInterface::apply_front_position(0);
            sleep(TIME_MS2I(elevator_check_interval));
        }

        sleep(TIME_MS2I(100));//

        // Step 4. Move forward to make the front wheels and rear assistant wheel to be on the stage
        chassis_target_vx = -1000;
        sleep(TIME_MS2I(500));
        chassis_target_vx = -300;
        sleep(TIME_MS2I(1200));

        chassis_target_vx = 0;

        sleep(TIME_MS2I(100));//

        // Step 5. Lift the rear wheels
//        chassis_target_vx = -1000;
        ElevatorInterface::apply_rear_position(0);
        while (ElevatorInterface::elevator_wheels[2].is_in_action() ||
               ElevatorInterface::elevator_wheels[3].is_in_action()) {
            ElevatorInterface::apply_rear_position(0);
            sleep(TIME_MS2I(elevator_check_interval));
        }
        chassis_target_vx = -1500;
        sleep(TIME_MS2I(1000));

        // Step 5. Move forward to make the rear wheels and be on the stage
//        chassis_target_vx = -1000;
//        sleep(TIME_MS2I(1500));
//        chassis_target_vx = -500;
//        sleep(TIME_MS2I(700));

        chassis_target_vx = 0;

        // Complete
        status = STOP;
        exit(0);
    }
}

