//
// Created by Wu Feiyang on 2023/7/6.
//
#include "ch.hpp"
#include "hal.h"
#include "can_motor_controller.h"
#include "can_motor_interface.h"
#include "shell.h"
#include "remote_interpreter.h"
#include "user_dart.h"
#include "led.h"
#include "referee_interface.h"

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

class printThread: public BaseStaticThread<512>{
private:
    CANMotorFeedback feedback;
    void main() final{
        setName("print");
        while(!shouldTerminate()){
            feedback = CANMotorIF::motor_feedback[CANMotorCFG::YAW];
            LOG("%f\n\r",feedback.accumulate_angle());
            chThdSleepMilliseconds(500);
        }
    }
}print_thread;

int main(){
    halInit();
    chibios_rt::System::init();
    Shell::start(LOWPRIO);
    Remote::start();

    LED::led_on(0); // number 0 led turned on now
    can1.start(NORMALPRIO+3);
    can2.start(NORMALPRIO+4);
    LED::led_on(1);// number 1 led turned on now

    /// Setup Referee
    Referee::init();

    UserDart::start(NORMALPRIO-1);
    CANMotorController::start(HIGHPRIO-1,HIGHPRIO-2,&can1,&can2);
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
