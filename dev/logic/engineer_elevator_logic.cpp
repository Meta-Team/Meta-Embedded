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
            EngineerElevatorSKD::set_target_height(ELEVATOR_ORIGIN_HEIGHT);
            EngineerElevatorSKD::set_aided_motor_velocity(0);
            EngineerChassisSKD::set_velocity(0, 0, 0);
            LOG("ELE STOP");
            break;
        case PREPARING:
            if (going_up) {
                EngineerChassisSKD::set_velocity(0, 0.10f * ENGINEER_CHASSIS_VELOCITY_MAX, 0);
            } else {
                EngineerChassisSKD::set_velocity(0, -0.05f * ENGINEER_CHASSIS_VELOCITY_MAX, 0);
            }
            LOG("ELE PREPARING");
            break;
        case ASCENDING:
            EngineerChassisSKD::set_velocity(0, 0, 0);
            EngineerElevatorSKD::set_target_height(STAGE_HEIGHT + ELEVATOR_ORIGIN_HEIGHT);
            LOG("ELE ASCENDING");
            break;
        case AIDING:
            if (going_up) {
                EngineerElevatorSKD::set_aided_motor_velocity(1.0f * ENGINEER_AIDED_MOTOR_VELOCITY);
            } else {
                EngineerElevatorSKD::set_aided_motor_velocity(-0.2f * ENGINEER_AIDED_MOTOR_VELOCITY);
            }
            LOG("ELE AIDING");
            break;
        case DESCENDING:
            EngineerElevatorSKD::set_target_height(ELEVATOR_ORIGIN_HEIGHT);
            EngineerElevatorSKD::set_aided_motor_velocity(0);
            LOG("ELE DESCENDING");
            break;
        case GIVING_BULLET:
            EngineerElevatorSKD::set_target_height(3.5f - ELEVATOR_ORIGIN_HEIGHT);
            LOG("ELE GIVING_BULLET");
            break;
    }
}

void EngineerElevatorLG::EngineerElevatorLGThread::main() {

    setName("ElevatorLG");

    while (!shouldTerminate()) {

        /*** update_hanging_status ***/
        // client lights are arranged in this way: [FL BL BR FR]
        // in chassis interface, motor_id : FR - 0, FL - 1, BL - 2, BR - 3
        // light the client lights when the wheel is hanging

        bool FL_hanging = !palReadPad(GPIOC, FR_SENSOR);
        bool BL_hanging = !palReadPad(GPIOC, BL_SENSOR);
        bool BR_hanging = !palReadPad(GPIOC, BR_SENSOR);
        bool FR_hanging = !palReadPad(GPIOC, FL_SENSOR);


        Referee::set_client_light(0, FL_hanging);
        Referee::set_client_light(1, BL_hanging);
        Referee::set_client_light(2, BR_hanging);
        Referee::set_client_light(3, FR_hanging);

//        bool sign = -1;

        if (!test_mode && auto_elevator) {

            switch (state) {

                case PREPARING:

                    if (going_up) {

                        if ((palReadPad(FF_SWITCH_PAD, FFL_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS)
                            && (palReadPad(FF_SWITCH_PAD, FFR_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS)) {
                            // Both the two sensors in front detect the stage, can start to go up-stair

                            set_state(ASCENDING);
                        }

                    } else {
                        if (BR_hanging && BL_hanging) {
                            // Both the two sensors at the back wheels reach the edge, can start to go down-stairs
                            set_state(ASCENDING);
                        }
                        else if ( BL_hanging && !BR_hanging )
                            EngineerChassisSKD::pivot_turn(- CHASSIS_WIDTH / 2, - CHASSIS_LENGTH / 2, -0.05 * ENGINEER_CHASSIS_W_MAX);
                        else if ( !BL_hanging && BR_hanging )
                            EngineerChassisSKD::pivot_turn(+ CHASSIS_WIDTH / 2, - CHASSIS_LENGTH / 2, +0.05 * ENGINEER_CHASSIS_W_MAX);

                    }
                    break;
                case ASCENDING:

                    if (ABS_IN_RANGE(STAGE_HEIGHT + ELEVATOR_ORIGIN_HEIGHT - EngineerElevatorSKD::get_current_height(), 0.5)) {
                        set_state(AIDING);
                    }

                    break;
                case AIDING:

                    if (going_up) {
                        if (!BR_hanging || !BL_hanging) {
                            // Both the two sensors at the back wheels landed on stage, enter the last step of going up-stairs
                            set_state(DESCENDING);
                        }

                    } else {

                        if (FL_hanging && FR_hanging) {
                            // Both the two sensors at the front wheels leave the stage, enter the last step of going down-stairs
                            set_state(DESCENDING);
                        }

                    }

                    break;
                case DESCENDING:

                    if (ABS_IN_RANGE(EngineerElevatorSKD::get_current_height() - ELEVATOR_ORIGIN_HEIGHT, 0.3)) {
                        set_state(STOP);
                    }

                    break;
                case GIVING_BULLET:

//                    sign = -sign;
//                    EngineerChassisSKD::pivot_turn(0, - CHASSIS_LENGTH / 2, sign * ENGINEER_CHASSIS_W_MAX);
//                    chThdSleepMilliseconds(500);

                    break;
                default:
                    break;
                case STOP:
                    break;
            }
        }

 //       LOG("FR, FL, BL, BR: %u %u %u %u", !FR_hanging, !FL_hanging, !BL_hanging, !BR_hanging);
        sleep(TIME_MS2I(ELEVATOR_LG_INTERVAL));
    }

}