//
// Created by 18767 on 2020/1/20.
//

#include "robotic_arm_logic.h"

//initializing static variables for memory space assignment

bool RoboticArmLG::auto_logic_enabled;
bool RoboticArmLG::grabbing;
RoboticArmLG::grab_state_t RoboticArmLG::grabState;
RoboticArmLG::action_plan_t RoboticArmLG::actionPlan[2];
RoboticArmLG::target_box_t * RoboticArmLG::targetBox = nullptr;      //current taarget
RoboticArmLG::command_info_t * RoboticArmLG::commandInfo = nullptr;  //current command
RoboticArmLG::command_list_t RoboticArmLG::commandList;
int RoboticArmLG::plan;

//definition of methods

void RoboticArmLG::init() {
    auto_logic_enabled = true;
    grabbing = false;
    grabState = FINISHED;
    actionPlan[0].num = 2;
    actionPlan[1].num = 4;
    commandList.num = 4;
    generate_plans();
    generate_command_list();
    using_plan(1); //use the first grabbing plan as default
    targetBox = actionPlan[plan].head;
    commandInfo = commandList.head;
    update_command_list();
}

void RoboticArmLG::start_grabbing() {
    grabbing = true;
}

void RoboticArmLG::finish() {
    NewRoboticArmSkd::set_clamp(false);
    NewRoboticArmSkd::set_x_slide_state(0);
    NewRoboticArmSkd::set_extension(false);
    NewRoboticArmSkd::set_motor_instruction(NewRoboticArmSkd::RETRIEVE);
    while (NewRoboticArmSkd::get_motor_state() != NewRoboticArmSkd::RETRIEVED ||
        NewRoboticArmSkd::is_clamped() || NewRoboticArmSkd::is_extended() || NewRoboticArmSkd::get_x_slide_state() != 0){}
    //the reset operation finished
    grabbing = false;
    targetBox = actionPlan[plan].head;
    commandInfo = commandList.head;
}

void RoboticArmLG::auto_logic_enable(bool enable) {
    auto_logic_enabled = enable;
}

void RoboticArmLG::using_plan(int val) {
    RoboticArmLG::plan = val - 1;
    finish();
    targetBox = actionPlan[plan].head;
    commandInfo = commandList.head;
    //now grabbing is false
    //once you switch the plan of grabbing, the engineer will not directly resume
}

void RoboticArmLG::generate_plans() {
    target_box_t * boxes = new target_box_t[6];
    for (int i = 0; i<6; i++){
        boxes[i].id = i+1;
        if (i>=0 && i<=2)
            boxes[i].x = i;
        else
            boxes[i].x = i-3;
        boxes[i].y = (i>=0 && i<=2) ? 0 : 1;
    }
    //plan1
    RoboticArmLG::actionPlan[0].head = &(boxes[0]);
    boxes[0].prev = &(boxes[0]);
    boxes[0].next = &(boxes[2]);
    boxes[2].prev = &(boxes[0]);
    boxes[2].next = nullptr;
    //plan2
    RoboticArmLG::actionPlan[1].head = &(boxes[3]);
    boxes[3].prev = &(boxes[3]);
    boxes[3].next = &(boxes[1]);
    boxes[1].prev = &(boxes[3]);
    boxes[1].next = &(boxes[4]);
    boxes[4].prev = &(boxes[1]);
    boxes[4].next = &(boxes[5]);
    boxes[5].prev = &(boxes[4]);
    boxes[5].next = nullptr;
}

void RoboticArmLG::generate_command_list() {
    command_info_t * cmds = new command_info_t[4];
    for (int i = 0; i<4; i++){
        cmds[i].slide_x = 0;
        cmds[i].slide_y = cmds[i].clamp = false;
        if (i == 1 || i == 2)
            cmds[i].clamp = true;
        if (i == 1 || i == 0)
            cmds[i].command = NewRoboticArmSkd::STRETCH;
        else
            cmds[i].command = NewRoboticArmSkd::RETRIEVE;
    }
    RoboticArmLG::commandList.head = &(cmds[0]);
    cmds[0].prev = &(cmds[0]);
    cmds[0].next = &(cmds[1]);
    cmds[1].prev = &(cmds[0]);
    cmds[1].next = &(cmds[2]);
    cmds[2].prev = &(cmds[1]);
    cmds[2].next = &(cmds[3]);
    cmds[3].prev = &(cmds[2]);
    cmds[3].next = nullptr;
}

void RoboticArmLG::update_command_list(){
    int x,y;
    x = targetBox->x;
    y = targetBox->y;
    //command1
    command_info_t * cmd = commandList.head;
    cmd->slide_x = x;
    cmd->slide_y = y;
    //command2
    cmd = cmd->next;
    cmd->slide_x = x;
    cmd->slide_y = y;
    //command 3 & 4
    cmd = cmd->next;
    cmd->slide_x = x;
    cmd = cmd->next;
    cmd->slide_x = x;
}

void RoboticArmLG::release_memory(RoboticArmLG::command_list_t cl, RoboticArmLG::action_plan_t * ap, int num_plans){
    //delete command list
    command_info_t * cmd = cl.head;
    command_info_t * temp;
    while (cmd != nullptr){
        temp = cmd;
        cmd = cmd->next;
        delete temp;
    }
    //delete plan
    for (int i = 0; i<num_plans; i++){
        target_box_t * box = ap[i].head;
        target_box_t * temp;
        while (box != nullptr){
            temp = box;
            box = box->next;
            delete temp;
        }
    }
}

void RoboticArmLG::next_step() {

}

void RoboticArmLG::previous_step() {

}

void RoboticArmLG::next_bin(){
    commandInfo = commandList.head;
    targetBox = targetBox->next;
    if (targetBox == nullptr)
        return;
    update_command_list();
}

void RoboticArmLG::previous_bin(){
    commandInfo = commandList.head;
    targetBox = targetBox->prev;
    if (targetBox == nullptr)
        return;
    update_command_list();
}

void RoboticArmLG::RoboticArmLGThread::main(){
    setName("RoboticArmLG thread");
    init();
    while (!shouldTerminate()) {
        if (auto_logic_enabled){
            // Auto logic
            if (grabbing){
                //next box?
                if (commandInfo->next == nullptr && NewRoboticArmSkd::get_motor_state() == NewRoboticArmSkd::RETRIEVED) {
                    //update command list
                    next_bin();
                    //end of grabbing plan?
                    if (targetBox == nullptr){
                        targetBox = actionPlan[plan].head;
                        commandInfo = commandList.head;
                        finish();
                    }
                }
                //next step: according to the command list as well as the current state
                if (commandInfo->next != nullptr){
                    //commandInfo is command 1, command 2, command 3
                    
                }
            //next loop
            }
        } else{
            // Manual logic
        }
    }
    release_memory(commandList,actionPlan,2);
}