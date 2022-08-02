//
// Created by Chen Qian on 11/18/21.
//

#include "ch.hpp"
#include "hal.h"

#include "interface/led/led.h"
#include "interface/virtual_COM/VirtualCOMPort.h"
#include "shell.h"
// Other headers here

using namespace chibios_rt;

SerialUSBDriver SDU;

class DataSendingThread : public BaseStaticThread<512> {
    void main() final {
        setName("DSendThd");
        while(!shouldTerminate()) {
            sleep(TIME_MS2I(100));
            Shell::printf("Buffer Content:" SHELL_NEWLINE_STR);
            for(int i = 0; i < 100; i++) {
                Shell::printf("%d ", VirtualCOMPort::rxbuffer[i]);
            }
            Shell::printf(SHELL_NEWLINE_STR);
            sleep(TIME_MS2I(100));
        }
    }
} dataSendingThread;

int main(void) {
    halInit();
    System::init();
    VirtualCOMPort::init(&SDU, NORMALPRIO+1);
    dataSendingThread.start(NORMALPRIO+5);
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(NORMALPRIO+10);

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
