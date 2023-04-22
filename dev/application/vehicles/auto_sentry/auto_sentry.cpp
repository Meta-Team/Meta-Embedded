//
// Created by 钱晨 on 11/14/21.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "interface/led/led.h"
#include "shell.h"
#include "chassis_logic.h"
#include "can_motor_controller.h"
#include "can_motor_interface.h"
#include "remote_interpreter.h"
#include "hardware_conf.h"
// Other headers here
#include "buzzer_scheduler.h"
//#define UT_CHASSIS_SHELL_CONTROL

using namespace chibios_rt;
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */


// Thread to ...
class ChassisControl : public BaseStaticThread <512> {
private:
    void main() final {
        setName("Control");
        while (!shouldTerminate()) {
            if(Remote::rc.s1 == Remote::S_UP) {
                ChassisLG::set_mode(ChassisLG::FORCE_RELAX_MODE);
                ChassisLG::set_target(0.0f, 0.0f);
                ChassisLG::set_target_omega(0.0f);
            } else if(Remote::rc.s1 == Remote::S_MIDDLE){
                ChassisLG::set_mode(ChassisLG::CHASSIS_REF_MODE);
                ChassisLG::set_target(Remote::rc.ch2 * 1500.0f, Remote::rc.ch3 * 1000.0f);
                ChassisLG::set_target_omega(Remote::rc.ch0 * 150.0f);
            }else if(Remote::rc.s1 == Remote::S_DOWN) {
                ChassisLG::set_mode(ChassisLG::CHASSIS_REF_MODE);
                ChassisLG::set_target_omega(300.f);
            }
            sleep(TIME_MS2I(100));
        }
    }
} ControlThread;
// tweak the speed of a specified motor
DEF_SHELL_CMD_START(MotorVelocity)
    if(argc != 2){
        return false;
    }
    LOG("Set motor%d's velocity:%.2f",(CANMotorCFG::motor_id_t)Shell::atoi(argv[0]), Shell::atof(argv[1]));
    CANMotorController::set_target_vel((CANMotorCFG::motor_id_t)Shell::atoi(argv[0]), Shell::atof(argv[1]));

    return true;
DEF_SHELL_CMD_END
// turn a specific motor on/off
DEF_SHELL_CMD_START(MotorFB)
    if(argc != 2){
        return false;
    }
    CANMotorController::shell_display((CANMotorCFG::motor_id_t)Shell::atoi(argv[0]),(CANMotorCFG::motor_id_t)Shell::atoi(argv[1]));
    return true;
DEF_SHELL_CMD_END
// switch to the mode of controlling each motor's speed in the shell console


const Shell::Command debugCmds[4] = {
        {"motorVelocity","motorVelocity [id0-3] [speed(float)]", MotorVelocity, nullptr},
        {"motorFB","motorFB [id0-3] [on/off0-1]", MotorFB, nullptr},
        {nullptr,nullptr,nullptr,nullptr}
};
int main() {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(debugCmds);
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO+1);
    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);
    Remote::start();
#ifndef UT_CHASSIS_SHELL_CONTROL
    MecanumChassisSKD::init(HIGHPRIO-4,550.0f,500.0f,478.0f);

    ChassisLG::init(NORMALPRIO+4, NORMALPRIO+5, 180.0f);
    ChassisLG::set_mode(ChassisLG::CHASSIS_REF_MODE);
    ControlThread.start(NORMALPRIO + 6);
    BuzzerSKD::init(NORMALPRIO);
    BuzzerSKD::play_sound(BuzzerSKD::Touhou15Stage5Boss);
#else
    CANMotorCFG::enable_v2i[CANMotorCFG::FL] = true;
    CANMotorCFG::enable_v2i[CANMotorCFG::FR] = true;
    CANMotorCFG::enable_v2i[CANMotorCFG::BR] = true;
    CANMotorCFG::enable_v2i[CANMotorCFG::BL] = true;
#endif


#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}
