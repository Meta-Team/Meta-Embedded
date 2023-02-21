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

int main(void) {
    halInit();
    System::init();

    // Start shell at high priority
    Shell::start(HIGHPRIO);
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO+1);
    CANMotorIF::init(&can1, &can2);

    CANMotorIF::set_current(CANMotorCFG::MOTOR1, 3000);

    return 0;
}