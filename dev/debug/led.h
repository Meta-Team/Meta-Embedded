#ifndef META_INFANTRY_LED_H
#define META_INFANTRY_LED_H

#include "ch.hpp"
#include "hal.h"

// Class for control LEDs

class LED {

public:

    static void green_off() {
        palSetPad(GPIOF, GPIOF_LED_GREEN);
    }

    static void green_on() {
        palClearPad(GPIOF, GPIOF_LED_GREEN);
    }

    static void green_toggle() {
        palTogglePad(GPIOF, GPIOF_LED_GREEN);
    }

    static void red_off() {
        palSetPad(GPIOF, GPIOF_LED_GREEN);
    }

    static void red_on() {
        palClearPad(GPIOF, GPIOF_LED_GREEN);
    }

    static void red_toggle() {
        palTogglePad(GPIOF, GPIOF_LED_GREEN);
    }
};

extern "C" {
#include "chconf.h"
// LED_RED_ON and LED_GREEN_OFF when halt.
void set_led_when_halt(void);
}

void init_led();

#endif