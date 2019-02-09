//
// Created by liuzikai on 2019-02-09.
//

#include "ch.hpp"
#include "hal.h"

#include "button_monitor.h"
#include "led.h"
#include "serial_shell.h"

using namespace chibios_rt;

/**
 * Blinks two LEDs, a demonstration of how to write a thread in C++.
 */
class BlinkLEDThread : public chibios_rt::BaseStaticThread<256> {
private:
    void main() final {
        setName("blink");
        while (!shouldTerminate()) {
            sleep(TIME_MS2I(100));
            if (buttonK0.pressed) {
                LED::red_on();
                sleep(TIME_MS2I((1 + buttonK0.counter % 3) * 100));
                LED::red_off();
                sleep(TIME_MS2I((1 + buttonK0.counter % 3) * 100));
            } else {
                LED::green_on();
                sleep(TIME_MS2I((1 + buttonK0.counter % 3) * 100));
                LED::green_off();
                sleep(TIME_MS2I((1 + buttonK0.counter % 3) * 100));
            }
        }
    }
} blinkLEDThread;

int main(void) {
    halInit();
    System::init();

    // Start button monitor threads.
    buttonK0.start(NORMALPRIO);

    // Start LED blink thread.
    blinkLEDThread.start(NORMALPRIO - 1);

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);


#if CH_CFG_NO_IDLE_THREAD // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while(true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif

    return 0;
}