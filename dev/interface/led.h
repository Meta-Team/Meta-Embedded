#ifndef META_INFANTRY_LED_H
#define META_INFANTRY_LED_H

#include "ch.hpp"
#include "hal.h"

// NOTICE: on both board, LED shares same pads and differ only in pins. So definition in board.h is enough
#if defined(BOARD_RM_2018_A)
// RM_BOARD_2018_A: Green - PF14, Red - PE11
#elif defined(BOARD_RM_2017)
// RM_BOARD_2017: Green - PF14, Red - PE7
#else
#error "LED has not been defined for selected board"
#endif

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
        palSetPad(GPIOE, GPIOE_LED_RED);
    }

    static void red_on() {
        palClearPad(GPIOE, GPIOE_LED_RED);
    }

    static void red_toggle() {
        palTogglePad(GPIOE, GPIOE_LED_RED);
    }
};

extern "C" {
#include "chconf.h"
// LED_RED_ON and LED_GREEN_OFF when halt.
void set_led_when_halt(void);
}

#endif