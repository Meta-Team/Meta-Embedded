//
// Created by Kerui Zhu on 7/15/2019.
//

#include "engineer_logic.h"

EngineerLogic::EngineerLogicThread EngineerLogic::engineerLogicThread;
EngineerLogic::engineer_state_t EngineerLogic::state;
bool EngineerLogic::elevated;
uint8_t EngineerLogic::edges;
uint16_t EngineerLogic::trigger_height;

void EngineerLogic::EngineerLogicThread::main() {
    setName("engineer_logic");
    state = FREE;
    elevated = false;
    edges = 0;
    DMSInterface::init(4);
    while (!shouldTerminate()){
        update_dms_data();

        sleep(5);
    }
}

void EngineerLogic::elevate_up() {
}

void EngineerLogic::elevate_down() {
}

void EngineerLogic::set_vx_direction(int v_id, int direction) {
    switch (v_id){

    }
}

void EngineerLogic::update_dms_data() {
    for (int i  = 0; i < 4; i++){
        bool is_edged;
        if ((is_edged = DMSInterface::get_distance(i) > trigger_height) ^ (edges & (1U << i))){
            if (is_edged){
                Referee::set_client_light(i, true);
                edges |= (1U << i);
            } else{
                Referee::set_client_light(i, false);
                edges &= ~(1U << i);
            }
        }
    }
    if (edges){
        if (edges == 15){
            // If the four vertices is hovering, then robot could only move forward
            state = FORWARD_ONLY;
            // If the robot is moving backwards, set the velocity to 0 immediately
            if (EngineerElevatorSKD::target_velocity[2] < 0) EngineerElevatorSKD::target_velocity[2] = EngineerElevatorSKD::target_velocity[3] = 0;
        } else if (edges == ){

        }
    } else if (elevated){
        state = LOCK;
    } else{
        state = FREE;
    }
}
