//
// Created by Chen Qian on 10/29/21.
//

/**
 * @file can_motor_scheduler.cpp
 * @brief Class to drive can motors.
 *
 * @addtogroup CAN Driver
 * @{
 */

#include "damiao_motor_controller.h"

DamiaoMotorController::feedbackThread DamiaoMotorController::FeedbackThread;
DamiaoMotorController::skdThread DamiaoMotorController::SKDThread;
DamiaoMotorFeedback DamiaoMotorIF::motor_feedback[DamiaoMotorCFG::MOTOR_COUNT];

void DamiaoMotorController::start(tprio_t SKD_PRIO, tprio_t FB_PRIO, CANInterface *can1_, CANInterface *can2_) {
    DamiaoMotorIF::init(can1_, can2_);
    SKDThread.start(SKD_PRIO);
    FeedbackThread.start(FB_PRIO);
}

void DamiaoMotorController::shell_display(DamiaoMotorCFG::MotorName name, bool enable) {
    FeedbackThread.enable_feedback[name] = enable;
}

void DamiaoMotorController::set_target_angle(DamiaoMotorCFG::MotorName name, float target) {
    target = degree2radius(target);
    DamiaoMotorIF::set_position(name,target);
}

void DamiaoMotorController::set_target_vel(DamiaoMotorCFG::MotorName name, float target) {
    DamiaoMotorIF::set_velocity(name,target);
}

float DamiaoMotorController::get_torque(DamiaoMotorCFG::MotorName name) {
    return DamiaoMotorIF::motor_feedback[name].actual_torque;
}

float DamiaoMotorController::get_position_angel(DamiaoMotorCFG::MotorName name) {
    return DamiaoMotorIF::motor_feedback[name].actual_angle;
}

int DamiaoMotorController::get_error(DamiaoMotorCFG::MotorName name) {
    return DamiaoMotorIF::motor_feedback[name].err_code;
}

float DamiaoMotorController::get_mos_temprature(DamiaoMotorCFG::MotorName name) {
    return DamiaoMotorIF::motor_feedback[name].mos_avg_tempr_raw;
}

float DamiaoMotorController::get_rotor_temprature(DamiaoMotorCFG::MotorName name) {
    return DamiaoMotorIF::motor_feedback[name].rotor_avg_tempr_raw;
}

int DamiaoMotorController::get_ID(DamiaoMotorCFG::MotorName name) {
    return DamiaoMotorIF::motor_feedback[name].rotor_avg_tempr_raw;
}

void DamiaoMotorController::motor_enable(DamiaoMotorCFG::MotorName name) {
    DamiaoMotorIF::start(name);
}

void DamiaoMotorController::motor_disable(DamiaoMotorCFG::MotorName name) {
    DamiaoMotorIF::stop(name);
}

void DamiaoMotorController::set_target_MIT(DamiaoMotorCFG::MotorName name,float pos,float vel,float torque) {
    if(DamiaoMotorIF::motors_mode[name]==MIT_MODE){
        chSysLock();
        DamiaoMotorIF::set_velocity(name,vel);
        DamiaoMotorIF::set_position(name,degree2radius(pos + DamiaoMotorCFG::motorCfg[name].initial_encoder_angle));
        DamiaoMotorIF::set_torque(name,torque);
        DamiaoMotorIF::set_param_MIT(name,DamiaoMotorCFG::motorCfg[name].mitKp,DamiaoMotorCFG::motorCfg[name].mitKd);
        chSysUnlock();
    }

}

void DamiaoMotorController::set_target_VEL(DamiaoMotorCFG::MotorName name, float vel) {
    if(DamiaoMotorIF::motors_mode[name]==VEL_MODE){
        chSysLock();
        DamiaoMotorIF::set_velocity(name,vel);
        chSysUnlock();
    }
}

void DamiaoMotorController::set_target_POSVEL(DamiaoMotorCFG::MotorName name, float pos, float vel) {
    if(DamiaoMotorIF::motors_mode[name]==POS_VEL_MODE){
        chSysLock();
        DamiaoMotorIF::set_velocity(name,vel);
        DamiaoMotorIF::set_position(name,degree2radius(pos + DamiaoMotorCFG::motorCfg[name].initial_encoder_angle));
        chSysUnlock();
    }
}

float DamiaoMotorController::degree2radius(float angle_degree) {
    return angle_degree /360.0f * 2 * PI;
}

void DamiaoMotorController::skdThread::main(){
    setName("SKD Thread");
    while(!shouldTerminate()){
        for(int i=0;i<DamiaoMotorCFG::MOTOR_COUNT;i++){
            DamiaoMotorIF::postMsg((DamiaoMotorCFG::MotorName)i);
        }
    }
    // This command will adjust the sleep time to ensure controller was triggered at a constant rate.
    // Sampling time could be critical when designing controller. Non-uniform sampling time would cause robustness problem in control system.
    tprio_t PRIO = this->getPriorityX();
    unsigned long sleep_time = THREAD_INTERVAL - (TIME_I2US(chVTGetSystemTimeX()) + PRIO)%THREAD_INTERVAL;
    sleep(TIME_US2I(sleep_time));
}
void DamiaoMotorController::feedbackThread::main() {
    setName("feedback");
    while(!shouldTerminate()) {
        for (int i = 0; i < DamiaoMotorCFG::MOTOR_COUNT; i++) {
            if(enable_feedback[i]){
//                LOG("Vel:%.2f,Pos:%.2f,Torque:%.2f" "Accumulate Angle:%2.f",DamiaoMotorIF::motor_feedback[i].actual_velocity,DamiaoMotorIF::motor_feedback[i].actual_angle/(2*PI)*360,
//                    DamiaoMotorIF::motor_feedback[i].actual_torque,DamiaoMotorIF::motor_feedback[i].accumulate_angle());
                LOG("Actual:%.2f, Accumulate:%.2f",DamiaoMotorIF::motor_feedback[i].actual_angle/(2*PI)*360,DamiaoMotorIF::motor_feedback[i].accumulate_angle()/(2*PI)*360);
            }
        }
        tprio_t PRIO = this->getPriorityX();
        unsigned long sleep_time = THREAD_INTERVAL - (TIME_I2US(chVTGetSystemTimeX()) + PRIO)%THREAD_INTERVAL;
        sleep(TIME_US2I(sleep_time));
    }
}

/** @} */