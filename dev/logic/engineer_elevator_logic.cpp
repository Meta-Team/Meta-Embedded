//
// Created by LaiXinyi on 7/15/2019.
//

#include "engineer_elevator_logic.h"

EngineerElevatorLG::EngineerElevatorLGThread EngineerElevatorLG::engineerLogicThread;
EngineerElevatorLG::action_t EngineerElevatorLG::action;
EngineerElevatorLG::action_t EngineerElevatorLG::prev_action;
EngineerElevatorLG::elevator_state_t EngineerElevatorLG::state;

bool EngineerElevatorLG::reach_stage;
bool EngineerElevatorLG::back_landed;
bool EngineerElevatorLG::back_edged;
bool EngineerElevatorLG::front_leave_stage;

uint16_t EngineerElevatorLG::hanging_trigger;
uint16_t EngineerElevatorLG::landed_trigger;

float EngineerElevatorLG::prev_e_tg_h;
float EngineerElevatorLG::prev_aR_tg_v;
float EngineerElevatorLG::prev_aL_tg_v;


void EngineerElevatorLG::init(tprio_t logic_thread_prio) {
    action = LOCK;
    prev_action = LOCK;
    state = STOP;
    reach_stage = false;
    back_landed = true;
    back_edged = false;
    front_leave_stage = false;
    hanging_trigger = 1800;     //TODO need to determine
    landed_trigger = 2300;      //TODO need to determine

    engineerLogicThread.start(logic_thread_prio);
}

void EngineerElevatorLG::set_action_free() {
    if (action == LOCK || action == PAUSE) {
        action = FREE;
        state = STOP;
        LOG("set free!");
    }
}

void EngineerElevatorLG::set_action_lock() {
    action = LOCK;
    state = STOP;
}

void EngineerElevatorLG::start_going_up() {
    if (action == LOCK && state == STOP)   { action = UPWARD;    going_up(); }
    else         LOG("cannot go up-stairs from start");
}

void EngineerElevatorLG::start_going_down() {
    if (action == LOCK && state == STOP)   { action = DOWNWARD;  going_down(); }
    else         LOG("cannot go down-stairs from start");
}


void EngineerElevatorLG::pause_action() {

    if (action == FREE) {
        LOG("pause from FREE");
        prev_action = FREE;
        action = PAUSE;
        prev_e_tg_h = EngineerElevatorSKD::target_height;
        prev_aR_tg_v = EngineerElevatorSKD::target_velocity[2];
        prev_aL_tg_v = EngineerElevatorSKD::target_velocity[3];
    }
    else if (action == UPWARD || action == DOWNWARD) {
        if (action == UPWARD)   LOG("pause_action from going UPWARD");
        if (action == DOWNWARD) LOG("pause_action from going DOWNWARD");
        prev_action = action;
        action = PAUSE;
        EngineerChassisSKD::lock();
    }
}

void EngineerElevatorLG::continue_action() {
    LOG("continue");
    if (action != PAUSE)
        LOG("invalid cont, elevator not paused");
    else {
        action = prev_action;
        prev_action = LOCK;

        if (action == FREE) {
            LOG("resume to FREE");
            EngineerElevatorSKD::elevator_enable(true);
            EngineerElevatorSKD::aided_motor_enable(true);
            EngineerElevatorSKD::target_height = prev_e_tg_h ;
            EngineerElevatorSKD::target_velocity[2] = prev_aR_tg_v ;
            EngineerElevatorSKD::target_velocity[3] = prev_aL_tg_v ;
        }
    }
}

void EngineerElevatorLG::quit_action() {

    LOG("quit");
    if (action != PAUSE)
        LOG("invalid quit, elevator not paused");
    else {
        if (prev_action != UPWARD && prev_action != DOWNWARD) {
            LOG("invalid quit, no prev_action");
            return;
        }
        else if (prev_action == UPWARD)     action = DOWNWARD;
        else if (prev_action == DOWNWARD)   action = UPWARD;

        // they share the same logic.
        prev_action = LOCK;
        switch (state) {
            case ASCENDING:     state = DESCENDING;     break;
            case AIDING:        state = AIDING;         break;
            case DESCENDING:    state = ASCENDING;      break;
            case PREPARING:
                action = LOCK;
                state = STOP;
                EngineerChassisSKD::unlock();
                break;
            default:            break;      // cannot be STOP
        }
    }

}



void EngineerElevatorLG::update_hanging_status() {

    // client lights are arranged in this way: [FL BL BR FR]
    // in chassis interface, motor_id : FR - 0, FL - 1, BL - 2, BR - 3
    // light the client lights when the wheel is hanging

    adcsample_t data[4];
    DMSInterface::get_raw_sample(data);

    bool FL_hanging = data[DMSInterface::FL] < hanging_trigger;
    bool BL_hanging = data[DMSInterface::BL] < hanging_trigger;
    bool BR_hanging = data[DMSInterface::BR] < hanging_trigger;
    bool FR_hanging = data[DMSInterface::FR] < hanging_trigger;

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
    /// STOP -> PREPARING -> ASCENDING -> AIDING -> DESCENDING -> STOP(LOCK)

    if (state == STOP) {
        LOG("preparing");
        state = PREPARING;
    }
    else if (state == PREPARING) {
        EngineerChassisSKD::unlock();
        EngineerElevatorSKD::elevator_enable(false);
        EngineerElevatorSKD::aided_motor_enable(false);
        EngineerChassisSKD::set_velocity(0, 0.05*ENGINEER_CHASSIS_VELOCITY_MAX, 0);  //TODO speed up?

        reach_stage = ( palReadPad(FF_SWITCH_PAD, FFL_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS )
                    && ( palReadPad(FF_SWITCH_PAD, FFR_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS ) ;

        if ( reach_stage ) {
            LOG("ascending");
            state = ASCENDING;
        }
    }
    else if (state == ASCENDING) {
//         reach_stage = false;
//         back_landed = false;
        EngineerChassisSKD::lock();
        EngineerElevatorSKD::elevator_enable(true);
        EngineerElevatorSKD::aided_motor_enable(false);
        EngineerElevatorSKD::set_target_height(STAGE_HEIGHT);

        if ( STAGE_HEIGHT - 0.5 <= EngineerElevatorIF::get_current_height() ) {
            LOG("aiding");
            state = AIDING;
        }
    }
    else if (state == AIDING) {
        EngineerChassisSKD::lock();
        EngineerElevatorSKD::elevator_enable(false);
        EngineerElevatorSKD::aided_motor_enable(true);
        EngineerElevatorSKD::set_aided_motor_velocity( 0.7*ENGINEER_AIDED_MOTOR_VELOCITY, 0.7*ENGINEER_AIDED_MOTOR_VELOCITY);

        adcsample_t data[4];
        DMSInterface::get_raw_sample(data);

        bool BL_landed = data[DMSInterface::BL] > landed_trigger;
        bool BR_landed = data[DMSInterface::BR] > landed_trigger;

        back_landed = BR_landed && BL_landed;

        if ( back_landed ) {
            LOG("descending");
            state = DESCENDING;
        }
    }
    else if (state == DESCENDING) {
        EngineerChassisSKD::lock();
        EngineerElevatorSKD::elevator_enable(true);
        EngineerElevatorSKD::aided_motor_enable(false);
        EngineerElevatorSKD::set_target_height(0);

        if ( 0.5 >= EngineerElevatorIF::get_current_height() ) {
            LOG("stop");
            state = STOP;
            EngineerChassisSKD::unlock();
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(false);
            action = LOCK;
            LOG("going up-stairs done");
        }
    }
}


void EngineerElevatorLG::going_down() {

    /// near the edge
    /// STOP -> PREPARING -> ASCENDING -> AIDING -> DESCENDING -> STOP(LOCK)

    if (state == STOP) {
        LOG("preparing");
        state = PREPARING;
    }
    else if (state == PREPARING) {
        EngineerChassisSKD::unlock();
        EngineerElevatorSKD::elevator_enable(false);
        EngineerElevatorSKD::aided_motor_enable(false);

        adcsample_t data[4];
        DMSInterface::get_raw_sample(data);
        bool BL_hanging = data[DMSInterface::BL] < hanging_trigger;
        bool BR_hanging = data[DMSInterface::BR] < hanging_trigger;

        back_edged = BR_hanging && BL_hanging;

        if ( back_edged ) {
            LOG("ascending");
            state = ASCENDING;
        }
        else if ( BL_hanging && !BR_hanging )
            EngineerChassisSKD::pivot_turn(EngineerChassisSKD::BL, -0.01 * ENGINEER_CHASSIS_W_MAX);
        else if ( !BL_hanging && BR_hanging )
            EngineerChassisSKD::pivot_turn(EngineerChassisSKD::BR, +0.01 * ENGINEER_CHASSIS_W_MAX);
        else
            EngineerChassisSKD::set_velocity(0, -0.02 *ENGINEER_CHASSIS_VELOCITY_MAX, 0);
    }
    else if (state == ASCENDING) {
//        back_edged = false;
//        front_leave_stage = false;
        EngineerChassisSKD::lock();
        EngineerElevatorSKD::elevator_enable(true);
        EngineerElevatorSKD::aided_motor_enable(false);
        EngineerElevatorSKD::set_target_height(STAGE_HEIGHT);

        if ( STAGE_HEIGHT - 0.7 <= EngineerElevatorIF::get_current_height() ) {
            LOG("aiding");
            state = AIDING;
        }
    }
    else if (state == AIDING) {
        EngineerChassisSKD::lock();
        EngineerElevatorSKD::elevator_enable(false);
        EngineerElevatorSKD::aided_motor_enable(true);
        EngineerElevatorSKD::set_aided_motor_velocity(-0.5*ENGINEER_AIDED_MOTOR_VELOCITY, -0.5*ENGINEER_AIDED_MOTOR_VELOCITY);

        adcsample_t data[4];
        DMSInterface::get_raw_sample(data);
        bool FL_hanging = data[DMSInterface::FL] < hanging_trigger;
        bool FR_hanging = data[DMSInterface::FR] < hanging_trigger;

        front_leave_stage = FL_hanging && FR_hanging ;

        if ( front_leave_stage ) {
            LOG("descending");
            state = DESCENDING;
        }
    }
    else if (state == DESCENDING) {
        EngineerChassisSKD::lock();
        EngineerElevatorSKD::elevator_enable(true);
        EngineerElevatorSKD::aided_motor_enable(false);
        EngineerElevatorSKD::set_target_height(0);

        if ( 0.3 >= EngineerElevatorIF::get_current_height() ) {
            LOG("stop");
            state = STOP;
            EngineerChassisSKD::unlock();
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(false);
            action = LOCK;
            LOG("going down-stairs done");
        }
    }

}


void EngineerElevatorLG::EngineerElevatorLGThread::main() {
    setName("ElevatorLG");

    DMSInterface::init(4);

    while (!shouldTerminate()) {

        update_hanging_status();

        if (action == LOCK || action == PAUSE) {
            EngineerElevatorSKD::elevator_enable(false);
            EngineerElevatorSKD::aided_motor_enable(false);
        } else if (action == UPWARD) {
            going_up();
        } else if (action == DOWNWARD) {
            going_down();
        } else if (action == FREE) {
            state = STOP;   // FREE is for debugging separately, not in any of the auto states
        }

        sleep(TIME_MS2I(ELEVATOR_LG_INTERVAL));

    }

}
