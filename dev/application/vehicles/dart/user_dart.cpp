//
// Created by Wu Feiyang on 2023/7/11.
//
#include "user_dart.h"
#include "damiao_motor_controller.h"
UserDart::UserThread UserDart::userThread;
bool UserDart::UserThread::start_flag;
bool UserDart::timer_started;

void UserDart::start(tprio_t user_thd_prio) {
    userThread.start(user_thd_prio);
}



void UserDart::UserThread::main(){
    setName("RemoteControl");
    static float angle;
    feedback = CANMotorIF::motor_feedback[CANMotorCFG::YAW];
    angle = feedback.accumulate_angle();

    Rudder::init();
    Rudder rudder1(&PWMD4,0,Rudder::MG995);
    Rudder rudder2(&PWMD4,1,Rudder::MG995);
    rudder1.enable();
    rudder2.enable();
    Rudder::start(&PWMD4, nullptr);
    rudder2.set_rudder_angle(0);
    start_flag = false;
    timer_started = false;
    time = 1;
    int start_time;
    int current_time;



    while(!shouldTerminate()){
//        DamiaoMotorController::set_target_POSVEL(DamiaoMotorCFG::DART_LOADER,90,2);
        if(Remote::rc.s1 == Remote::S_UP){                  // Forced Relax Mode
            chSysLock();
            for(uint i=0;i<CANMotorCFG::MOTOR_COUNT;i++){
                CANMotorCFG::enable_a2v[i] = false;
                CANMotorCFG::enable_v2i[i] = false;
            }
            CANMotorController::set_target_current(CANMotorCFG::YAW,0);
            CANMotorController::set_target_current(CANMotorCFG::STORE_ENERGY_LEFT,0);
            CANMotorController::set_target_current(CANMotorCFG::STORE_ENERGY_RIGHT,0);
            chSysUnlock();
        }else if(Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP){        // Remote contorl mode
            chSysLock();
            CANMotorCFG::enable_v2i[CANMotorCFG::YAW] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::YAW] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::TRIGGER_ADJUST] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::TRIGGER_ADJUST] = false;
            CANMotorController::set_target_vel(CANMotorCFG::YAW, 200*Remote::rc.ch2);
            CANMotorController::set_target_vel(CANMotorCFG::STORE_ENERGY_LEFT,-500*Remote::rc.ch3);
            CANMotorController::set_target_vel(CANMotorCFG::STORE_ENERGY_RIGHT,500*Remote::rc.ch3);
            CANMotorController::set_target_vel(CANMotorCFG::TRIGGER_ADJUST,2000*Remote::rc.ch1);
            chSysUnlock();
            rudder1.set_rudder_angle(180 - (int)Remote::rc.ch0 * 180);

        }else if((Remote::rc.s1 == Remote::S_DOWN && Remote::rc.s2 == Remote::S_UP) || Remote::key.f){          // PC control mode, goes back to initial encoder angle

            CANMotorCFG::enable_v2i[CANMotorCFG::YAW] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::YAW] = true;
            CANMotorController::set_target_angle(CANMotorCFG::YAW,angle);
            uint8_t status = Referee::dart_client.dart_launch_opening_status;
            if(status == 0){
                if(Remote::rc.wheel > 0.2){
                    start_flag = true;
                }
                if(time == 0){ // first time of launch.
                    if(start_flag){
                        if(!timer_started){
                            start_time = TIME_I2MS(chVTGetSystemTimeX());
                            timer_started = true;
                        }
                        current_time = TIME_I2MS(chVTGetSystemTimeX());
                        if(current_time - start_time < 2000){
                            CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_LEFT,3900);
                            CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_RIGHT,-3900);
                            rudder1.set_rudder_angle(180);
                            chThdSleepMilliseconds(5);
                            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = false;
                            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = false;
                            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
                            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
                            chThdSleepMilliseconds(5);
                            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = true;
                            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = true;
                            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = true;
                            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = true;
                        }else if(current_time - start_time >= 2000 && current_time - start_time < 4000){
                            CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_LEFT,0);
                            CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_RIGHT,0);
                        }else if(current_time -start_time >=4000 && current_time - start_time < 4500) {
                            rudder1.set_rudder_angle(0);
                        }else if(current_time -start_time >=4500 && current_time - start_time < 5000){
                            rudder1.set_rudder_angle(180);
                        }else if(current_time > 5000){
                            start_flag = false;
                            timer_started = false;
                            time++;
                        }
                    }
                }
            }else if(time==1){
                if(start_flag){
                    if(!timer_started){
                        start_time = TIME_I2MS(chVTGetSystemTimeX());
                        timer_started = true;
                    }
                    current_time = TIME_I2MS(chVTGetSystemTimeX());
                    if(current_time - start_time < 2000){
                        CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_LEFT,3900);
                        CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_RIGHT,-3900);
                        rudder1.set_rudder_angle(180);
                        chThdSleepMilliseconds(5);
                        CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = false;
                        CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = false;
                        CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
                        CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
                        chThdSleepMilliseconds(5);
                        CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = true;
                        CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = true;
                        CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = true;
                        CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = true;
                    }else if(current_time - start_time >= 2000 && current_time - start_time < 4000){
                        CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_LEFT,0);
                        CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_RIGHT,0);
                        DamiaoMotorController::set_target_POSVEL(DamiaoMotorCFG::DART_LOADER,90,4);
                    }else if(current_time - start_time >= 4000 && current_time -start_time < 5500){
                        rudder2.set_rudder_angle(10);
                    }else if(current_time - start_time >= 5500 && current_time -start_time < 6000) {
                        rudder2.set_rudder_angle(0);
                    }else if(current_time - start_time >= 6000 && current_time -start_time < 7000){
                        DamiaoMotorController::set_target_POSVEL(DamiaoMotorCFG::DART_LOADER,0,4);
                    }else if(current_time -start_time >= 7000 && current_time - start_time < 7500){
                        rudder1.set_rudder_angle(180);
                    }else if(current_time -start_time >= 7500 && current_time - start_time < 8000){
                        rudder1.set_rudder_angle(0);
                    }else if(current_time > 8000){
                        start_flag = false;
                        timer_started = false;
                        time++;
                    }
                }
            }else if(time == 2){
                if(start_flag){
                    if(!timer_started){
                        start_time = TIME_I2MS(chVTGetSystemTimeX());
                        timer_started = true;
                    }
                    current_time = TIME_I2MS(chVTGetSystemTimeX());
                    if(current_time - start_time < 2000){
                        CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_LEFT,3900);
                        CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_RIGHT,-3900);
                        rudder1.set_rudder_angle(180);
                        chThdSleepMilliseconds(5);
                        CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = false;
                        CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = false;
                        CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
                        CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
                        chThdSleepMilliseconds(5);
                        CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = true;
                        CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = true;
                        CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = true;
                        CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = true;
                    }else if(current_time - start_time >= 2000 && current_time - start_time < 4000){
                        CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_LEFT,0);
                        CANMotorController::set_target_angle(CANMotorCFG::STORE_ENERGY_RIGHT,0);
                        DamiaoMotorController::set_target_POSVEL(DamiaoMotorCFG::DART_LOADER,-90,4);
                    }else if(current_time - start_time >= 4000 && current_time -start_time < 5500){
                        rudder2.set_rudder_angle(10);
                    }else if(current_time - start_time >= 5500 && current_time -start_time < 6000) {
                        rudder2.set_rudder_angle(0);
                    }else if(current_time - start_time >= 6000 && current_time -start_time < 7000){
                        DamiaoMotorController::set_target_POSVEL(DamiaoMotorCFG::DART_LOADER,0,4);
                    }else if(current_time -start_time >= 7000 && current_time - start_time < 7500){
                        rudder1.set_rudder_angle(180);
                    }else if(current_time -start_time >= 7500 && current_time - start_time < 8000){
                        rudder1.set_rudder_angle(0);
                    }else if(current_time > 8000){
                        start_flag = false;
                        timer_started = false;
                        time++;
                    }
                }
            }
        }else if(Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE){
            chSysLock();
            CANMotorCFG::enable_v2i[CANMotorCFG::YAW] = true;
            CANMotorCFG::enable_a2v[CANMotorCFG::YAW] = true;
            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_LEFT] = false;
            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_LEFT] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
            CANMotorCFG::enable_a2v[CANMotorCFG::STORE_ENERGY_RIGHT] = false;
            CANMotorController::set_target_angle(CANMotorCFG::YAW,angle);
            chSysUnlock();
        }


//        feedback = CANMotorIF::motor_feedback[CANMotorCFG::YAW];
//        LOG("initial angle: %f, current angle: %f\r\n",angle,feedback.accumulate_angle());
        chThdSleepMilliseconds(100);
    }
}