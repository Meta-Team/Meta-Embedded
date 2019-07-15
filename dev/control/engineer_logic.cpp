//
// Created by Kerui Zhu on 7/15/2019.
//

#include "engineer_logic.h"

EngineerLogic::EngineerLogicThread EngineerLogic::engineerLogicThread;
EngineerLogic::engineer_state_t EngineerLogic::state;
int16_t EngineerLogic::dms_heights[4];
uint8_t EngineerLogic::edges;
uint16_t EngineerLogic::trigger_height;

void EngineerLogic::EngineerLogicThread::main() {
    setName("engineer_logic");
    state = FREE;
    edges = 0;
    DMSInterface::init(4);
    for (int i = 0; i < 4; i++)
        dms_heights[i] = DMSInterface::get_distance(i);
    while (!shouldTerminate()){
        if (DMSInterface::get_distance(FR))
        sleep(5);
    }
}

void EngineerLogic::elevate_up() {
}

void EngineerLogic::elevate_down() {
}

void EngineerLogic::set_vx_direction(int v_id, int direction) {
    switch (v_id)
}
