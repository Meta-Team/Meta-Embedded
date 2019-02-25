//
// Created by liuzikai on 2019-02-24.
//

#include "elevator_thread.h"
#include "buzzer.h"

ElevatorThread::status_t ElevatorThread::get_status() {
    return status_;
}

bool ElevatorThread::start_up_actions(tprio_t prio) {
    if (status_ != STOP) return false;
    status_ = UPWARD;
    chibios_rt::BaseStaticThread<ELEVATOR_THREAD_WORKING_AREA_SIZE>::start(prio);
    return true;
}

bool ElevatorThread::start_down_actions(tprio_t prio) {
    if (status_ != STOP) return false;
    status_ = DOWNWARD;
    chibios_rt::BaseStaticThread<ELEVATOR_THREAD_WORKING_AREA_SIZE>::start(prio);
    return true;
}

void ElevatorThread::emergency_stop() {
    status_ = STOP;
    exit(0xFF); // FIXME: this doesn't work since it terminate the call thread
}

float ElevatorThread::get_chassis_target_vy() {
    return chassis_target_vy_;
}

void ElevatorThread::main() {
    setName("elevator");

    if (status_ == STOP) {
        exit(-1);
    } else if (status_ == UPWARD) {

        /** Step 1. Lower all wheels **/
        ElevatorInterface::apply_front_position(-stage_height_);
        ElevatorInterface::apply_rear_position(-stage_height_);
        // FIXME: FIGURE OUT WHY WHEEL 0 STOP FEEDBACK
        while (/*ElevatorInterface::elevator_wheels[0].is_in_action() ||*/
               ElevatorInterface::elevator_wheels[1].is_in_action() ||
               ElevatorInterface::elevator_wheels[2].is_in_action() ||
               ElevatorInterface::elevator_wheels[3].is_in_action()) {
            ElevatorInterface::apply_front_position(-stage_height_);
            ElevatorInterface::apply_rear_position(-stage_height_);
            sleep(TIME_MS2I(elevator_check_interval_));
        }

        sleep(TIME_MS2I(100)); // Time interval between steps

        /** Step 2. Move forward to make the front assistant wheels to get onto the stage **/
        chassis_target_vy_ = -500;
        sleep(TIME_MS2I(500));

        chassis_target_vy_ = 0;
        sleep(TIME_MS2I(100));//

        /** Step 3. Lift the front wheels **/
        ElevatorInterface::apply_front_position(0);
        // FIXME: FIGURE OUT WHY WHEEL 0 STOP FEEDBACK
        while (/*ElevatorInterface::elevator_wheels[0].is_in_action() ||*/
               ElevatorInterface::elevator_wheels[1].is_in_action()) {
            ElevatorInterface::apply_front_position(0);
            sleep(TIME_MS2I(elevator_check_interval_));
        }

        sleep(TIME_MS2I(100));  // Time interval between steps

        /** Step 4. Move forward to make the front wheels and rear assistant wheel to be on the stage **/
        chassis_target_vy_ = -1000;
        sleep(TIME_MS2I(500));
        chassis_target_vy_ = -300;
        sleep(TIME_MS2I(1200));

        chassis_target_vy_ = 0;

        sleep(TIME_MS2I(100));  // Time interval between steps

        /** Step 5. Lift the rear wheels **/
        ElevatorInterface::apply_rear_position(0);
        while (ElevatorInterface::elevator_wheels[2].is_in_action() ||
               ElevatorInterface::elevator_wheels[3].is_in_action()) {
            ElevatorInterface::apply_rear_position(0);
            sleep(TIME_MS2I(elevator_check_interval_));
        }
        chassis_target_vy_ = -1500;
        sleep(TIME_MS2I(1000));

        chassis_target_vy_ = 0;

        /** Complete **/
        status_ = STOP;
        exit(0);
    } 
    // TODO: write action of going downward
}

