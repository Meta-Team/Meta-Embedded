//
// Created by 18767 on 2020/1/20.
//

#include "engineer_auto_logic.h"

int engineer_auto_logic::grabMode;

void engineer_auto_logic::grab_mode1() {
    if (RoboticArmSKD::state == RoboticArmSKD::NORMAL){
        //the basic plan is to use the extend_state and the clamp state to control the whole process
    }
}

void engineer_auto_logic::set_mode(int mode_){
    grabMode = mode_;
    RoboticArmSKD::should_set_arm_normal = 0;
}

void engineer_auto_logic::stop_using_arm() {
    RoboticArmSKD::should_set_arm_normal = 1;
}
void engineer_auto_logic::EngineerAutoThread::main(){
    while (!shouldTerminate()) {
        if (grabMode != none){
            if (grabMode == mode1)
                grab_mode1();
            else if (grabMode == mode2)
                grab_mode2();
        }
    }
}