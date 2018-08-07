//
// Created by liuzikai on 2018/8/6.
//
#include "unit_test_common.hpp"

void UnitTestBlinkLEDThread::main(void) {
    // Initialize GPIO to the LEDs.
    palSetPadMode(GPIOF, 14, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOE, 7, PAL_MODE_OUTPUT_PUSHPULL);
    // Set the name of this thread
    setName("unit_test_led");
    // Blink the leds
    while(!shouldTerminate()) {
        LED_RED_ON;
        LED_GREEN_OFF;
        sleep(TIME_MS2I(2000));
        LED_RED_OFF;
        LED_GREEN_ON;
        sleep(TIME_MS2I(2000));
    }
}