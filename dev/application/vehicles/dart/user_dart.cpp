//
// Created by Wu Feiyang on 2023/7/11.
//
#include "user_dart.h"
UserDart::UserThread UserDart::userThread;
void UserDart::start(tprio_t user_thd_prio) {
    userThread.start(user_thd_prio);
}

void UserDart::UserThread::main(){
    setName("RemoteControl");
    static float angle;
    feedback = CANMotorIF::motor_feedback[CANMotorCFG::MOTOR_ONE];
    angle = feedback.accumulate_angle();
    while(!shouldTerminate()){
        chSysLock();
        if(Remote::rc.s1 == Remote::S_UP){
            CANMotorCFG::enable_v2i[CANMotorCFG::MOTOR_ONE] = false;
            CANMotorCFG::enable_a2v[CANMotorCFG::MOTOR_ONE] = false;
            CANMotorController::set_target_current(CANMotorCFG::MOTOR_ONE,0);
        }else if(Remote::rc.s1 == Remote::S_MIDDLE){
            CANMotorCFG::enable_v2i[CANMotorCFG::MOTOR_ONE] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::MOTOR_ONE] = false;
            CANMotorController::set_target_vel(CANMotorCFG::MOTOR_ONE, 200*Remote::rc.ch0);
        }else if(Remote::rc.s1 == Remote::S_DOWN){
            CANMotorCFG::enable_v2i[CANMotorCFG::MOTOR_ONE] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::MOTOR_ONE] = true;
            CANMotorController::set_target_angle(CANMotorCFG::MOTOR_ONE,angle);
        }

        chSysUnlock();
        feedback = CANMotorIF::motor_feedback[CANMotorCFG::MOTOR_ONE];
        LOG("initial angle: %f, current angle: %f\r\n",angle,feedback.accumulate_angle());
        chThdSleepMilliseconds(100);
    }
}