//
// Created by liuzikai on 2019-02-09.
//

#include "ch.hpp"
#include "hal.h"

#include "button_monitor.h"
#include "led.h"
#include "debug/shell/shell.h"

using namespace chibios_rt;

#if defined(BOARD_RM_2017)
ButtonMonitorThread buttonK0(GPIOD, 10);
#elif defined(BOARD_RM_2018_A)
ButtonMonitorThread buttonK0(GPIOB, 2);
#endif

/**
 * Blinks two LEDs, a demonstration of how to write a thread in C++.
 */
class BlinkLEDThread : public chibios_rt::BaseStaticThread<256> {
public:
    void set_button_monitor(ButtonMonitorThread* bmt) {
        button = bmt;
    }
private:

    ButtonMonitorThread* button = nullptr;

#if defined(BOARD_RM_2018_A)
    unsigned led_i = 1;
#endif

    void main() final {
        setName("blink");
        while (!shouldTerminate()) {
            sleep(TIME_MS2I(100));
            if (button->pressed) {
                LED::red_on();
                sleep(TIME_MS2I((1 + button->counter % 3) * 100));
                LED::red_off();
                sleep(TIME_MS2I((1 + button->counter % 3) * 100));
            } else {
                LED::green_on();
                sleep(TIME_MS2I((1 + button->counter % 3) * 100));
                LED::green_off();
                sleep(TIME_MS2I((1 + button->counter % 3) * 100));
            }
#if defined(BOARD_RM_2018_A)
            LED::led_off(led_i);
            led_i++;
            if (led_i > 8) led_i = 1;
            LED::led_on(led_i);
#endif
        }
    }
} blinkLEDThread;

int main(void) {
    halInit();
    System::init();

    // Start button monitor threads.
    buttonK0.start(NORMALPRIO);

    // Start LED blink thread.
    blinkLEDThread.set_button_monitor(&buttonK0);
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