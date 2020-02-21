//
// Created by 18767 on 2020/1/20.
//

#include "robotic_arm_logic.h"

bool RoboticArmLG::auto_logic_enabled;
bool RoboticArmLG::grabing;

void RoboticArmLG::init() {
    auto_logic_enabled = true;
    grabing = false;
}

void RoboticArmLG::get_prepared() {
    NewRoboticArmSkd::set_clamp(false);
    NewRoboticArmSkd::set_x_slide_state(0);
    NewRoboticArmSkd::set_extension(false);
    NewRoboticArmSkd::set_motor_instruction(NewRoboticArmSkd::STRECH);
    grabing = false;
}

void RoboticArmLG::start_grabing() {
    grabing = true;
}

void RoboticArmLG::finish() {
    NewRoboticArmSkd::set_clamp(false);
    NewRoboticArmSkd::set_x_slide_state(0);
    NewRoboticArmSkd::set_extension(false);
    NewRoboticArmSkd::set_motor_instruction(NewRoboticArmSkd::RETRIEVE);
    grabing = false;
}

void RoboticArmLG::auto_logic_enable(bool enable) {
    auto_logic_enabled = enable;
}

void RoboticArmLG::next_step() {

}

void RoboticArmLG::previous_step() {

}

void RoboticArmLG::RoboticArmLGThread::main(){
    setName("RoboticArmLG thread");
    init();
    while (!shouldTerminate()) {
        if (auto_logic_enabled){
            // Auto logic
            if (grabing){

            }
        } else{
            // Manual logic
        }
    }
}