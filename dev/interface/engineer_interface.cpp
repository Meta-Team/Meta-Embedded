//
// Created by zzb on 2020/1/18.
//

#include "engineer_interface.h"
#include "can_interface.h"
float EngineerInterface::present_angle[MOTOR_COUNT] = {0};
float EngineerInterface::present_velocity[MOTOR_COUNT] = {0};
int16_t EngineerInterface::target_current[MOTOR_COUNT] = {0};
CANInterface* EngineerInterface::can = nullptr;

void EngineerInterface::init(){

}

int EngineerInterface::data_to_can(){

}

void EngineerInterface::process_feedback_arm(){

}

void EngineerInterface::process_feedback_rescue(){

}


