//
// Created by LaiXinyi on 7/15/2019.
//

#include "engineer_elevator_logic.h"

EngineerElevatorLG::EngineerElevatorLGThread EngineerElevatorLG::engineerLogicThread;
EngineerElevatorLG::action_t EngineerElevatorLG::action;
EngineerElevatorLG::elevator_state_t EngineerElevatorLG::state;
bool EngineerElevatorLG::reach_stage;
bool EngineerElevatorLG::back_landed;
bool EngineerElevatorLG::back_edged;
bool EngineerElevatorLG::front_leave_stage;
float EngineerElevatorLG::reach_stage_trigger;
uint16_t EngineerElevatorLG::hanging_trigger;
uint16_t EngineerElevatorLG::landed_trigger;


void EngineerElevatorLG::init() {
    set_action(FREE);
    state = STOP;
    reach_stage = false;
    back_landed = false;
    back_edged = false;
    front_leave_stage = false;

    hanging_trigger = 2000;
    landed_trigger = 3000;

    //TODO the following values not determined yet
    reach_stage_trigger = 2000;

}


void EngineerElevatorLG::set_action(EngineerElevatorLG::action_t act) {

    // highest priority for LOCK
    if (act == LOCK) {
        LOG("forced stop, all freeze now");
        action = LOCK;
    }

    // when current action is LOCK or FREE, it is safe to set any action
    else if (action == LOCK || action == FREE)
        action = act;
    else if (action == UPWARD)
        LOG("set_action failed, current state: UPWARD");
    else if (action == DOWNWARD)
        LOG("set_action failed, current state: DOWNWARD");

}


void EngineerElevatorLG::update_hanging_status() {

    // client lights are arranged in this way: [FL BL BR FR]
    // in chassis interface, motor_id : FR - 0, FL - 1, BL - 2, BR - 3
    // light the client lights when the wheel is hanging

    bool FL_hanging = DMSInterface::get_raw_sample(DMSInterface::FL) < hanging_trigger;
    bool BL_hanging = DMSInterface::get_raw_sample(DMSInterface::BL) < hanging_trigger;
    bool BR_hanging = DMSInterface::get_raw_sample(DMSInterface::BR) < hanging_trigger;
    bool FR_hanging = DMSInterface::get_raw_sample(DMSInterface::FR) < hanging_trigger;

    // FL
    if ( FL_hanging )   Referee::set_client_light(0, true);
    else                Referee::set_client_light(0, false);
    // BL
    if ( BL_hanging )   Referee::set_client_light(1, true);
    else                Referee::set_client_light(1, false);
    // BR
    if ( BR_hanging )   Referee::set_client_light(2, true);
    else                Referee::set_client_light(2, false);
    // FR
    if ( FR_hanging )   Referee::set_client_light(3, true);
    else                Referee::set_client_light(3, false);

}


void EngineerElevatorLG::going_up() {

    /// (near the stage)
    /// STOP -> PREPARING -> ASCENDING -> AIDING -> DESCENDING -> free

    if (state == STOP) {
        state = PREPARING;
        LOG("preparing");
        EngineerChassisSKD::unlock();
        EngineerChassisSKD::set_velocity(0, 0.5*ENGINEER_CHASSIS_VELOCITY_MAX, 0);
    }
    else if (state == PREPARING) {
        //TODO
        //reach_stage = DMSInterface::get_distance(FFL) > reach_stage_trigger && DMSInterface::get_raw_sample(FFR) > reach_stage_trigger;
        if ( reach_stage ) {
            state = ASCENDING;
            LOG("ascending");
            EngineerChassisSKD::lock();
            EngineerElevatorSKD::elevator_enable(true);
            EngineerElevatorSKD::aided_motor_enable(false);
            EngineerElevatorSKD::set_target_height(STAGE_HEIGHT);
        }
    }
    else if (state == ASCENDING) {
        reach_stage = false;
        back_landed = false;
        if ( STAGE_HEIGHT - 0.05 <= EngineerElevatorIF::get_current_height() ) {
            state = AIDING;
            LOG("aiding");
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(true);
            EngineerElevatorSKD::set_aided_motor_velocity(ENGINEER_AIDED_MOTOR_VELOCITY, ENGINEER_AIDED_MOTOR_VELOCITY);
        }
    }
    else if (state == AIDING) {
        bool BL_landed = DMSInterface::get_raw_sample(DMSInterface::BL) > landed_trigger;
        bool BR_landed = DMSInterface::get_raw_sample(DMSInterface::BR) > landed_trigger;
        back_landed = BL_landed && BR_landed;

        if ( back_landed ) {
            state = DESCENDING;
            LOG("descending");
            EngineerElevatorSKD::elevator_enable(true);
            EngineerElevatorSKD::aided_motor_enable(false);
            EngineerElevatorSKD::set_target_height(0);
        }
    }
    else if (state == DESCENDING) {
        if ( 0.05 >= EngineerElevatorIF::get_current_height() ) {
            state = STOP;
            LOG("stop");
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(false);
            action = LOCK;
            LOG("going up-stairs done");
        }
    }
}


void EngineerElevatorLG::going_down() {

    /// near the edge
    /// STOP -> PREPARING -> ASCENDING -> AIDING -> DESCENDING -> free

    if (state == STOP) {
        state = PREPARING;
        LOG("preparing");
        EngineerChassisSKD::unlock();
        EngineerChassisSKD::set_velocity(0, -0.5*ENGINEER_CHASSIS_VELOCITY_MAX, 0);
    }
    else if (state == PREPARING) {
        bool BL_hanging = DMSInterface::get_raw_sample(DMSInterface::BL) < hanging_trigger;
        bool BR_hanging = DMSInterface::get_raw_sample(DMSInterface::BR) < hanging_trigger;
        back_edged = BL_hanging && BR_hanging;

        if ( back_edged ) {
            state = ASCENDING;
            LOG("ascending");
            EngineerChassisSKD::lock();
            EngineerElevatorSKD::elevator_enable(true);
            EngineerElevatorSKD::aided_motor_enable(false);
            EngineerElevatorSKD::set_target_height(STAGE_HEIGHT);
        }
        else if ( BL_hanging && !BR_hanging ) {
            EngineerChassisSKD::pivot_turn(BL, 0.5 * ENGINEER_CHASSIS_W_MAX);
        }
        else if ( !BL_hanging && BR_hanging ) {
            EngineerChassisSKD::pivot_turn(BR, -0.5 * ENGINEER_CHASSIS_VELOCITY_MAX);
        }
    }
    else if (state == ASCENDING) {
        back_edged = false;
        front_leave_stage = false;
        if ( STAGE_HEIGHT - 0.05 <= EngineerElevatorIF::get_current_height() ) {
            state = AIDING;
            LOG("aiding");
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(true);
            EngineerElevatorSKD::set_aided_motor_velocity(-ENGINEER_AIDED_MOTOR_VELOCITY, -ENGINEER_AIDED_MOTOR_VELOCITY);
        }
    }
    else if (state == AIDING) {
        bool FL_hanging = DMSInterface::get_raw_sample(DMSInterface::FL) < hanging_trigger;
        bool FR_hanging = DMSInterface::get_raw_sample(DMSInterface::FR) < hanging_trigger;
        front_leave_stage = FL_hanging && FR_hanging;

        if ( front_leave_stage ) {
            state = DESCENDING;
            LOG("descending");
            EngineerElevatorSKD::elevator_enable(true);
            EngineerElevatorSKD::aided_motor_enable(false);
            EngineerElevatorSKD::set_target_height(0);
        }
    }
    else if (state == DESCENDING) {
        if ( 0 >= EngineerElevatorIF::get_current_height() ) {
            state = STOP;
            LOG("stop");
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(false);
            action = LOCK;
            LOG("going down-stairs done");
        }
    }

}


//TODO force stop


void EngineerElevatorLG::EngineerElevatorLGThread::main() {
    setName("engineer_elevator_logic");

    DMSInterface::init(4);
    EngineerElevatorLG::init();

    while (!shouldTerminate()) {

        update_hanging_status();

        if (action == LOCK) {
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
