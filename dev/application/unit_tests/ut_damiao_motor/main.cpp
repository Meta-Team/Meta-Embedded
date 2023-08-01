//
// Created by Wu Feiyang on 2023/7/23.
//

#include "hal.h"
#include "ch.hpp"
#include "can_interface.h"
#include "shell.h"
#include "damiao_motor_feedback.h"
#include "damiao_motor_controller.h"

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

static CANConfig can_cfg = {
        CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
        CAN_BTR_SJW(0) | CAN_BTR_TS2(3) |
        CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
};


static CANTxFrame can_tx_frame[2];

static uint8_t ctr_id;
static uint8_t err;
static uint16_t pos;
static uint16_t vel;
static uint16_t torque;
static uint8_t mos_avg_tempr;
static uint8_t rotor_avg_tempr;

int main(){
    halInit();
    chibios_rt::System::init();
    can1.start(HIGHPRIO);
    can2.start(HIGHPRIO-1);
    Shell::start(NORMALPRIO+2);
//    print_thread.start(NORMALPRIO+4);
    DamiaoMotorController::start(NORMALPRIO+1,NORMALPRIO+2,&can1,&can2);
    DamiaoMotorController::motor_enable(DamiaoMotorCFG::YAW);
//    DamiaoMotorController::set_target_vel(DamiaoMotorCFG::YAW,5);.
//    DamiaoMotorController::set_target_angle(DamiaoMotorCFG::YAW,50);
//    DamiaoMotorController::set_target_POSVEL(DamiaoMotorCFG::YAW,20,2);
    DamiaoMotorController::shell_display(DamiaoMotorCFG::YAW,true);
//    DamiaoMotorController::set_target_MIT(DamiaoMotorCFG::YAW,2.5,0.9,0.9);
    DamiaoMotorController::set_target_POSVEL(DamiaoMotorCFG::YAW,0,2);
    chThdSleepSeconds(5);
    DamiaoMotorController::set_target_POSVEL(DamiaoMotorCFG::YAW,90,2);
    chThdSleepSeconds(5);
    DamiaoMotorController::set_target_POSVEL(DamiaoMotorCFG::YAW,-90,2);
    chThdSleepSeconds(5);
    DamiaoMotorController::set_target_POSVEL(DamiaoMotorCFG::YAW,0,2);
    chThdSleepSeconds(5);
    DamiaoMotorController::motor_disable(DamiaoMotorCFG::YAW);



}