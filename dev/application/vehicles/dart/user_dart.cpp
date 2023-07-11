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
    feedback = CANMotorIF::motor_feedback[CANMotorCFG::YAW];
    angle = feedback.accumulate_angle();
    while(!shouldTerminate()){
        chSysLock();
        if(Remote::rc.s1 == Remote::S_UP){                  // Forced Relax Mode
            CANMotorCFG::enable_v2i[CANMotorCFG::YAW] = false;
            CANMotorCFG::enable_a2v[CANMotorCFG::YAW] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = false;
            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
            CANMotorController::set_target_current(CANMotorCFG::YAW,0);
        }else if(Remote::rc.s1 == Remote::S_MIDDLE){        // Remote contorl mode
            CANMotorCFG::enable_v2i[CANMotorCFG::YAW] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::YAW] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
            CANMotorController::set_target_vel(CANMotorCFG::YAW, 200*Remote::rc.ch2);
            CANMotorController::set_target_vel(CANMotorCFG::STORE_ENERGY_LEFT,500*Remote::rc.ch3);
            CANMotorController::set_target_vel(CANMotorCFG::STORE_ENERGY_RIGHT,500*Remote::rc.ch3);
        }else if(Remote::rc.s1 == Remote::S_DOWN){          // PC control mode, goes back to initial encoder angle
            CANMotorCFG::enable_v2i[CANMotorCFG::YAW] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::YAW] = true;
            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = false;
            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
            CANMotorController::set_target_angle(CANMotorCFG::YAW,angle);
            uint8_t remain_time = Referee::ext_dart_remaining_time.dart_remaining_time;
            if(remain_time == 0){
                CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = true;
                CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = true;
                CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = true;
                CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = true;
            }
        }

        chSysUnlock();
        feedback = CANMotorIF::motor_feedback[CANMotorCFG::YAW];
        LOG("initial angle: %f, current angle: %f\r\n",angle,feedback.accumulate_angle());
        chThdSleepMilliseconds(100);
    }
}