//
// Created by Wu Feiyang on 2023/76.
//
#include "ch.hpp"
#include "hal.h"
#include "can_motor_controller.h"
#include "can_motor_interface.h"
#include "chprintf.h"
#include "shell.h"
#include "led.h"
#include "remote_interpreter.h"

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

class remoteThread:public BaseStaticThread<512>{
private:
    void main() final;
}remote_thread;

void remoteThread::main(){
    setName("RemoteControl");
    static uint16_t angle = CANMotorIF::motor_feedback[CANMotorCFG::MOTOR_ONE].last_rotor_angle_raw;
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
            CANMotorCFG::enable_v2i[CANMotorCFG::MOTOR_ONE] = true;
            CANMotorController::set_target_angle(CANMotorCFG::MOTOR_ONE,0 );
        }
        chSysUnlock();
        LOG("initial angle: %d, current angle: %d\r\n",angle,CANMotorIF::motor_feedback[CANMotorCFG::MOTOR_ONE].last_rotor_angle_raw);
        chThdSleepMilliseconds(100);
    }
}

class printThread: public BaseStaticThread<512>{
private:
    CANMotorFeedback feedback;
    void main() final{
        setName("print");
        while(!shouldTerminate()){
            feedback = CANMotorIF::motor_feedback[CANMotorCFG::MOTOR_ONE];
            LOG("%f\n\r",feedback.accumulate_angle());
            chprintf((BaseSequentialStream*)&USBSerialIF::SDU,"hello\r\n");
            chThdSleepMilliseconds(500);
        }

    }
}print_thread;

int main(){
    halInit();
    chibios_rt::System::init();
    Shell::start(LOWPRIO);
    Remote::start();
    can1.start(NORMALPRIO+3);
    can2.start(NORMALPRIO+4);
    remote_thread.start(NORMALPRIO-1);

//    CANMotorCFG::enable_v2i[CANMotorCFG::MOTOR_ONE] = true;
    CANMotorController::start(HIGHPRIO-1,HIGHPRIO-2,&can1,&can2);

    print_thread.start(HIGHPRIO);
//    CANMotorController::set_target_vel(CANMotorCFG::MOTOR_ONE,1000);
    chThdSleepSeconds(5);

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
