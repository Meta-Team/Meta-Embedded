#ifndef META_INFANTRY_LED_H
#define META_INFANTRY_LED_H

#include "ch.hpp"
#include "hal.h"

// NOTICE: On both board, LEDs share same pads and differ only in pins. So changing definitions in board.h is enough
#if defined(BOARD_RM_2018_A)
// Green - PF14, Red - PE11, LED 1-8 - PG 1-8. Low to turn on
#define LED_PAD_ON(port, pad) palClearPad(port, pad)
#define LED_PAD_OFF(port, pad) palSetPad(port, pad)
#elif defined(BOARD_RM_2017)
// Green - PF14, Red - PE7. High to turn on
#define LED_PAD_ON(port, pad) palSetPad(port, pad)
#define LED_PAD_OFF(port, pad) palClearPad(port, pad)
#else
#error "LED has not been defined for selected board"
#endif


/**
 * @name LED
 * @brief LED Controller
 * @note RM Board 2018 has additional 8 green LED
 */
class LED {

public:

    static void green_off() {
        LED_PAD_OFF(GPIOF, GPIOF_LED_GREEN);
    }

    static void green_on() {
        LED_PAD_ON(GPIOF, GPIOF_LED_GREEN);
    }

    static void green_toggle() {
        palTogglePad(GPIOF, GPIOF_LED_GREEN);
    }

    static void red_off() {
        LED_PAD_OFF(GPIOE, GPIOE_LED_RED);
    }

    static void red_on() {
        LED_PAD_ON(GPIOE, GPIOE_LED_RED);
    }

    static void red_toggle() {
        palTogglePad(GPIOE, GPIOE_LED_RED);
    }

#if defined(BOARD_RM_2018_A)
    static void led_on(unsigned i) {
        if (i >= 1 && i <= 8) LED_PAD_ON(GPIOG, i);
    }

    static void led_off(unsigned i) {
        if (i >= 1 && i <= 8) LED_PAD_OFF(GPIOG, i);
    }

    static void led_toggle(unsigned i) {
        if (i >= 1 && i <= 8) palTogglePad(GPIOG, i);
    }
#else
    static void led_on(unsigned i) {}

    static void led_off(unsigned i) {}

    static void led_toggle(unsigned i) {}
#endif

    static void all_off() {
        green_off();
        red_off();
#if defined(BOARD_RM_2018_A)
        for (unsigned i = 1; i <= 8; i++) {
            led_off(i);
        }
#endif
    }

};

extern "C" {
#include "chconf.h"
// LED_RED_ON and LED_GREEN_OFF when halt.
void set_led_when_halt(void);
}

#endif