//
// Created by LaiXinyi on 7/15/2019.
// Modified by Kerui Zhu on 7/28/2019
//

#include "engineer_elevator_logic.h"

bool EngineerElevatorLG::test_mode;
EngineerElevatorLG::EngineerElevatorLGThread EngineerElevatorLG::engineerLogicThread;
EngineerElevatorLG::elevator_state_t EngineerElevatorLG::state;
bool EngineerElevatorLG::auto_elevator;

bool EngineerElevatorLG::going_up = true;


void EngineerElevatorLG::init(tprio_t logic_thread_prio) {
    test_mode = true;
    state = STOP;
    auto_elevator = true;

    engineerLogicThread.start(logic_thread_prio);
}

EngineerElevatorLG::elevator_state_t EngineerElevatorLG::get_current_state() {
    return state;
}

void EngineerElevatorLG::change_auto_status() {
    auto_elevator = !auto_elevator;
};

void EngineerElevatorLG::set_test_mode(bool test_mode_) {
    test_mode = test_mode_;
}

void EngineerElevatorLG::set_elevate_dir(bool going_up_) {

    if (state == STOP) {  // normal

        going_up = going_up_;
        set_state(PREPARING);

    } else {  // abnormal state

        if (going_up_ == going_up) {

            set_state((elevator_state_t) ((state + 1) % 5));

        } else {  // going_up_ != going_up

            going_up = !going_up;  // reverse direction

            if (state == PREPARING) {
                set_state(STOP);
            } else if (state == ASCENDING) {
                set_state(DESCENDING);
            } else if (state == DESCENDING) {
                set_state(ASCENDING);
            } else if (state == AIDING) {
                set_state(AIDING);
            }
        }

    }

}

void EngineerElevatorLG::set_elevator_height(float new_height) {
    EngineerElevatorSKD::set_target_height(new_height);
}

void EngineerElevatorLG::give_bullet() {
    if (state == GIVING_BULLET)
        set_state(STOP);
    else {
//        chThdSleepMilliseconds(2000);
        set_state(GIVING_BULLET);
    }
}


void EngineerElevatorLG::set_state(EngineerElevatorLG::elevator_state_t new_state) {
    state = new_state;
    switch (state) {
        case STOP:

            break;
        case PREPARING:
            break;
        case ASCENDING:

            break;
        case AIDING:

            break;
        case DESCENDING:

            break;
        case GIVING_BULLET:

            break;
    }
}

void EngineerElevatorLG::EngineerElevatorLGThread::main() {

    setName("ElevatorLG");

    while (!shouldTerminate()) {

        /*** update_hanging_status ***/

        sleep(TIME_MS2I(ELEVATOR_LG_INTERVAL));
    }

}