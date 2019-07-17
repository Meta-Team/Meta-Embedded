//
// Created by Kerui Zhu on 7/15/2019.
//

#include "engineer_elevator_logic.h"

EngineerElevatorLG::EngineerElevatorLGThread EngineerElevatorLG::engineerLogicThread;
EngineerElevatorLG::action_t EngineerElevatorLG::action;
EngineerElevatorLG::elevator_state_t EngineerElevatorLG::state;


void EngineerElevatorLG::init() {
    //TODO
    set_action(FREE);
    state = STOP;
}


void EngineerElevatorLG::set_action(EngineerElevatorLG::action_t act) {

    // when current action is FREE, it is safe to set any action
    if (action == FREE)
        action = act;
    else if (act == FREE) {
        LOG("forced stop, all freeze now");
        set_action(FREE);
        //TODO
    } else {
        LOG("set_action failed");
        if (action == UPWARD)    LOG("current state: UPWARD");
        if (action == DOWNWARD)  LOG("current state: DOWNWARD");
    }

}


void EngineerElevatorLG::update_hanging_status() {

    /// client lights are arranged in this way: [FL BL BR FR]
    /// in chassis interface, motor_id : FR - 0, FL - 1, BL - 2, BR - 3

    for (unsigned i=0; i<4; i++) {
        if ( DMSInterface::check_hanging(i) )
            Referee::set_client_light( (i+3)%4 ,true);
        else
            Referee::set_client_light( (i+3)%4 , false);
    }
}


void EngineerElevatorLG::going_up() {

    /// (the aided motors already on the stage)
    /// start_going_up -> STOP -> ASCENDING -> AIDING -> DESCENDING -> free

    if (state == STOP) {
        state = ASCENDING;
        EngineerElevatorSKD::elevator_enable(true);
        EngineerElevatorSKD::aided_motor_enable(false);
        EngineerElevatorSKD::set_target_height(STAGE_HEIGHT);
    }
    else if (state == ASCENDING) {
        if ( STAGE_HEIGHT <= EngineerElevatorIF::get_current_height() ) {
            state = AIDING;
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(true);
            //TODO maybe allow to speed up?
            EngineerElevatorSKD::set_aided_motor_velocity(ENGINEER_AIDED_MOTOR_VELOCITY, ENGINEER_AIDED_MOTOR_VELOCITY);
        }
    }
    else if (state == AIDING) {
        if ( // (!DMSInterface::check_hanging(0)) && (!DMSInterface::check_hanging(1)) &&
             (!DMSInterface::check_hanging(2)) && (!DMSInterface::check_hanging(3))  )
        {   // when the back wheels are landed
            state = DESCENDING;
            EngineerElevatorSKD::elevator_enable(true);
            EngineerElevatorSKD::aided_motor_enable(false);
            EngineerElevatorSKD::set_target_height(0);
        }
        //TODO ele_pivot_rotate
    }
    else if (state == DESCENDING) {
        if ( 0 >= EngineerElevatorIF::get_current_height() ) {
            state = STOP;
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(false);
            action = FREE;
            LOG("going up-stairs done");
        }
    }
}


void EngineerElevatorLG::going_down() {

    /// the aided wheels already out of the stage, the back sensors just hanging
    /// start_going_down -> STOP -> ASCENDING -> AIDING -> DESCENDING -> free

    if (state == STOP) {
        if ( (!DMSInterface::check_hanging(2)) && (!DMSInterface::check_hanging(3)) ) {
            // both back wheels are still on the stages
            LOG("not ready yet, keep reversing");
            set_action(FREE);
        }
        else if ( DMSInterface::check_hanging(2) && DMSInterface::check_hanging(3) ) {
            // both back wheels are hanging
            state = ASCENDING;
            EngineerElevatorSKD::elevator_enable(true);
            EngineerElevatorSKD::aided_motor_enable(false);
            EngineerElevatorSKD::set_target_height(STAGE_HEIGHT);
        }
        else {
            // one of the wheel is hanging while the other is not
            //TODO cha_pivot_rotate
        }
    }
    else if (state == ASCENDING) {
        if ( STAGE_HEIGHT <= EngineerElevatorIF::get_current_height() ) {
            state = AIDING;
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(true);
            //TODO maybe allow to speed up?
            EngineerElevatorSKD::set_aided_motor_velocity(-ENGINEER_AIDED_MOTOR_VELOCITY, -ENGINEER_AIDED_MOTOR_VELOCITY);
        }
    }
    else if (state == AIDING) {
        if ( DMSInterface::check_hanging(0) && DMSInterface::check_hanging(1) )
        {   // when the front wheels leave the stage
            state = DESCENDING;
            EngineerElevatorSKD::elevator_enable(true);
            EngineerElevatorSKD::aided_motor_enable(false);
            EngineerElevatorSKD::set_target_height(0);
        }
        //TODO ele_pivot_rotate
    }
    else if (state == DESCENDING) {
        if ( 0 >= EngineerElevatorIF::get_current_height() ) {
            state = STOP;
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(false);
            action = FREE;
            LOG("going down-stairs done");
        }
    }

}




//TODO pivot_turn

//TODO quit action

//TODO other error checking




void EngineerElevatorLG::EngineerElevatorLGThread::main() {
    setName("engineer_elevator_logic");

    DMSInterface::init(4);
    EngineerElevatorLG::init();

    while (!shouldTerminate()) {

        update_hanging_status();

        if (action == FREE) {
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(false);
        } else if (action == UPWARD) {
            going_up();
        } else if (action == DOWNWARD) {
            going_down();
        }

        sleep(TIME_MS2I(ELEVATOR_LG_INTERVAL));

    }

}
