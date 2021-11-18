//
// Created by Chen Qian on 11/18/21.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "VCP.h"
#include "shell.h"
// Other headers here

using namespace chibios_rt;

SerialUSBDriver SDU;

int main(void) {
    halInit();
    System::init();
    VCP::init(&SDU);

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(NORMALPRIO+10, (BaseSequentialStream*)&SDU);

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
