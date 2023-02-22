//
// Created by Tianyi Han on 2/20/2023.
//

#include "ch.hpp"
#include "hal.h"

#include "interface/led/led.h"
#include "shell.h"
#include "can_motor_interface.h"
#include "hardware_conf.h"

using namespace chibios_rt;
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

class thd :public BaseStaticThread<512> {
    void main() final{
        setName("feedback");
        while(!shouldTerminate()){
            Shell::printf("%f", CANMotorIF::motor_feedback[CANMotorCFG::MOTOR1].actual_angle);
            CANMotorIF::set_current(CANMotorCFG::MOTOR1, 2000);
            CANMotorIF::post_target_current(CANMotorBase::can_channel_1, 0x1FF);
            sleep(TIME_MS2I(1));
        }
    }
}thddd;

int main(void) {
    halInit();
    System::init();

    // Start shell at high priority
    Shell::start(HIGHPRIO);
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO+1);
    CANMotorIF::init(&can1, &can2);
    chThdSleepMilliseconds(500);
    // turn green LED on
    LED::green_on();
    LED::red_off();
    LED::blue_off();
    thddd.start(NORMALPRIO+2);


#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}