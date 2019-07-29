//
// Created by LaiXinyi on 7/15/2019.
// Modified by Kerui Zhu on 7/28/2019
//

#include "engineer_elevator_logic.h"

bool EngineerElevatorLG::test_mode;
EngineerElevatorLG::EngineerElevatorLGThread EngineerElevatorLG::engineerLogicThread;
EngineerElevatorLG::elevator_state_t EngineerElevatorLG::state;

/*
bool EngineerElevatorLG::a_t_forward;
bool EngineerElevatorLG::a_t_backward;
uint32_t EngineerElevatorLG::delay_time;
*/
bool EngineerElevatorLG::pause;
float EngineerElevatorLG::pause_height;
bool EngineerElevatorLG::going_up_;
bool EngineerElevatorLG::auto_elevating;


void EngineerElevatorLG::init(tprio_t logic_thread_prio) {
    test_mode = true;
    state = STOP;
/*
    a_t_backward = false;
    a_t_forward = false;
    delay_time = 0;
*/
    pause = false;
    pause_height = 0;
    going_up_ = true;
    auto_elevating = false;

    set_auto_elevating(false);

    engineerLogicThread.start(logic_thread_prio);
}

void EngineerElevatorLG::set_auto_elevating(bool auto_elevating_) {
    auto_elevating = auto_elevating_;
}

void EngineerElevatorLG::going_up() {
    if (!pause){
        pause_action();
    } else{
        if (going_up_) next_step();
        else if (!going_up_ && state == PREPARING){
            LOG("stop");
            state = STOP;
        }else if (!going_up_ && state == ASCENDING) {
            LOG("descending");
            state = DESCENDING;
        }else if (!going_up_ && state == DESCENDING) {
            LOG("ascending");
            state = ASCENDING;
        }

        going_up_ = true;
        continue_action();
    }
}

void EngineerElevatorLG::going_down() {
    if (!pause){
        pause_action();
    } else{
        if (!going_up_) next_step();
        else if (going_up_ && state == PREPARING){
            LOG("stop");
            state = STOP;
        }else if (going_up_ && state == ASCENDING) {
            LOG("descending");
            state = DESCENDING;
        }else if (going_up_ && state == DESCENDING) {
            LOG("ascending");
            state = ASCENDING;
        }
    }

    going_up_ = false;
    continue_action();
}

void EngineerElevatorLG::pause_action() {
    LOG("pause");
    pause = true;
    pause_height = EngineerElevatorIF::get_current_height();
}

void EngineerElevatorLG::continue_action() {
    LOG("continue");
    pause = false;
}

void EngineerElevatorLG::next_step() {
    /**
     * On-Stage or Off-Stage Logic
     * (near the stage)
     * STOP -> PREPARING -> ASCENDING -> AIDING -> DESCENDING -> STOP
     */
     pause_action();

     if(state == STOP){
         state = PREPARING;
         LOG("preparing");
     }else if (state == PREPARING) {
         state = ASCENDING;
         LOG("ascending");
     }else if (state == ASCENDING) {
         state = AIDING;
         LOG("aiding");
     }else if (state == AIDING) {
         state = DESCENDING;
         LOG("descending");
     }else if (state == DESCENDING) {
         state = STOP;
         LOG("stop");
     }

     continue_action();
}

void EngineerElevatorLG::elevator_enable(bool enable_) {
    EngineerElevatorSKD::elevator_enable(enable_);
    EngineerElevatorSKD::aided_motor_enable(enable_);
}

void EngineerElevatorLG::set_elevator_height(float new_height) {
    EngineerElevatorSKD::set_target_height(new_height);
}

float EngineerElevatorLG::get_elevator_height() {
    return EngineerElevatorSKD::get_current_height();
}

void EngineerElevatorLG::set_aided_motor_velocity(float target_velocity) {
    EngineerElevatorSKD::set_aided_motor_velocity(target_velocity);
}


/////////// for finding better aided motor params
//void EngineerElevatorLG::aided_motor_test_forward() {
//    adcsample_t data[4];
//    DMSInterface::get_raw_sample(data);
//    bool back_landed = data[DMSInterface::BL] > landed_trigger && data[DMSInterface::BR] > landed_trigger;
//
//    if (!back_landed)
//        EngineerElevatorSKD::set_aided_motor_velocity(0.7*ENGINEER_AIDED_MOTOR_VELOCITY, 0.7*ENGINEER_AIDED_MOTOR_VELOCITY);
//    else
//        EngineerElevatorSKD::set_aided_motor_velocity(0,0);
//
//}
//
//void EngineerElevatorLG::aided_motor_test_backward() {
//    adcsample_t data[4];
//    DMSInterface::get_raw_sample(data);
//    bool front_leave_stage = data[DMSInterface::FL] < hanging_trigger && data[DMSInterface::FR] < hanging_trigger;
//
//    if (!front_leave_stage)
//        EngineerElevatorSKD::set_aided_motor_velocity(-0.5*ENGINEER_AIDED_MOTOR_VELOCITY, -0.5*ENGINEER_AIDED_MOTOR_VELOCITY);
//    else {
//        chThdSleepMilliseconds(delay_time);
//        EngineerElevatorSKD::set_aided_motor_velocity(0, 0);
//    }
//}


void EngineerElevatorLG::EngineerElevatorLGThread::main() {
    setName("ElevatorLG");

    DMSInterface::init(4);

    while (!shouldTerminate()) {

        /*** update_hanging_status ***/

        adcsample_t data[4];
        DMSInterface::get_raw_sample(data);

        // client lights are arranged in this way: [FL BL BR FR]
        // in chassis interface, motor_id : FR - 0, FL - 1, BL - 2, BR - 3
        // light the client lights when the wheel is hanging

        bool FL_hanging = data[DMSInterface::FL] < hanging_trigger;
        bool BL_hanging = data[DMSInterface::BL] < hanging_trigger;
        bool BR_hanging = data[DMSInterface::BR] < hanging_trigger;
        bool FR_hanging = data[DMSInterface::FR] < hanging_trigger;

        // FL
        Referee::set_client_light(0, FL_hanging);
        // BL
        Referee::set_client_light(1, BL_hanging);
        // BR
        Referee::set_client_light(2, BR_hanging);
        // FR
        Referee::set_client_light(3, FR_hanging);

        if (!test_mode){

            if (pause){
                EngineerElevatorSKD::set_target_height(pause_height);
                EngineerElevatorSKD::set_aided_motor_velocity(0);
                EngineerChassisSKD::set_velocity(0,0,0);
            } else{
                /*** update elevator status ***/

                bool reach_edge;

                switch (state){

                    case PREPARING:

                        if (going_up_){
                            // both the two sensors in front detect the stage, can start to go up-stairs
                            reach_edge = ( palReadPad(FF_SWITCH_PAD, FFL_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS )
                                         && ( palReadPad(FF_SWITCH_PAD, FFR_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS ) ;

                            if ( reach_edge ) {
                                pause_action();
                                if (auto_elevating) next_step();
                            } else{
                                EngineerChassisSKD::set_velocity(0, 0.05*ENGINEER_CHASSIS_VELOCITY_MAX, 0);
                            }
                        } else{

                            // FIXME: change to &&
                            // both the two sensors at the back wheels reach the edge, can start to go down-stairs
                            reach_edge = BR_hanging && BL_hanging;

                            if ( reach_edge ) {
                                pause_action();
                                if (auto_elevating) next_step();
                            }
                                // FIXME: re-enable
                                /*else if ( BL_hanging && !BR_hanging )
                                    EngineerChassisSKD::pivot_turn(EngineerChassisSKD::BL, -0.005 * ENGINEER_CHASSIS_W_MAX);
                                else if ( !BL_hanging && BR_hanging )
                                    EngineerChassisSKD::pivot_turn(EngineerChassisSKD::BR, +0.005 * ENGINEER_CHASSIS_W_MAX);
                                    */
                            else {
                                EngineerChassisSKD::set_velocity(0, -0.02 *ENGINEER_CHASSIS_VELOCITY_MAX, 0);
                            }
                        }
                        break;

                    case ASCENDING:
                        EngineerElevatorSKD::set_target_height(STAGE_HEIGHT);

                        if ( ABS_IN_RANGE(STAGE_HEIGHT - EngineerElevatorSKD::get_current_height(), 0.5)) {
                            if (auto_elevating) next_step();
                        }
                        break;

                    case AIDING:

                        if (going_up_){
                            EngineerElevatorSKD::set_aided_motor_velocity( 0.7 * ENGINEER_AIDED_MOTOR_VELOCITY);

                            // both the two sensors at the back wheels landed on stage, enter the last step of going up-stairs
                            reach_edge = data[DMSInterface::BR] > landed_trigger || data[DMSInterface::BL] > landed_trigger;

                        } else{
                            EngineerElevatorSKD::set_aided_motor_velocity(- 0.5 * ENGINEER_AIDED_MOTOR_VELOCITY);

                            // both the two sensors at the front wheels leave the stage, enter the last step of going down-stairs
                            reach_edge = FL_hanging && FR_hanging ;

                        }

                        if ( reach_edge ) {
                            pause_action();
                            if (auto_elevating) next_step();
                        }
                        break;

                    case DESCENDING:

                        EngineerElevatorSKD::set_target_height(0);

                        if (ABS_IN_RANGE(EngineerElevatorSKD::get_current_height(), 0.3)) {
                            next_step();
                        }
                        break;

                    case STOP:
                        EngineerElevatorSKD::set_target_height(0);
                    default:
                        break;
                }
            }
        }

        sleep(TIME_MS2I(ELEVATOR_LG_INTERVAL));

    }

}