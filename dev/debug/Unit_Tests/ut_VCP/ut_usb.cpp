//
// Created by Chen Qian on 11/18/21.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "interface/virtual_COM/VCP.h"
#include "shell.h"
// Other headers here

using namespace chibios_rt;

SerialUSBDriver SDU;

class DataSendingThread : public BaseStaticThread<512> {
    void main() final {
        setName("DSendThd");
        while(!shouldTerminate()) {
            int16_t senddata = -0x1221;
            int16_t senddata2 = 0x1221;
            uint8_t txdata[5] = {0xFF,(uint8_t)(senddata >> 8), (uint8_t)senddata,(uint8_t)(senddata2 >> 8), (uint8_t)senddata2};
            //VCP::sendData(txdata, 5);
            sleep(TIME_MS2I(100));
            Shell::printf("Buffer Content:" SHELL_NEWLINE_STR);
            for(int i = 0; i < 100; i++) {
                Shell::printf("%d ", VCP::buffer[i]);
            }
            sleep(TIME_MS2I(100));
        }
    }
} dataSendingThread;

int main(void) {
    halInit();
    System::init();
    VCP::init(&SDU, NORMALPRIO+1);
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
