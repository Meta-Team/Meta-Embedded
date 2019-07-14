/**
 * @file    led.h
 * @brief   Interface to operate on-board LEDs.
 *
 * @addtogroup LED
 * @{
 */

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
 * @note RM Board 2018 has additional 8 green LEDs
 */
class LED {

public:

    /**
     * Turn off green LED
     */
    static void green_off() {
        LED_PAD_OFF(GPIOF, GPIOF_LED_GREEN);
    }

    /**
     * Turn on green LED
     */
    static void green_on() {
        LED_PAD_ON(GPIOF, GPIOF_LED_GREEN);
    }

    /**
     * Toggle green LED
     */
    static void green_toggle() {
        palTogglePad(GPIOF, GPIOF_LED_GREEN);
    }

    /**
     * Turn off red LED
     */
    static void red_off() {
        LED_PAD_OFF(GPIOE, GPIOE_LED_RED);
    }

    /**
     * Turn on red LED
     */
    static void red_on() {
        LED_PAD_ON(GPIOE, GPIOE_LED_RED);
    }

    /**
     * Toggle red LED
     */
    static void red_toggle() {
        palTogglePad(GPIOE, GPIOE_LED_RED);
    }

#if defined(BOARD_RM_2018_A)

    /**
     * Turn on indexed green LED on RM Board 2018 A
     * @param i   LED index [1-8]
     */
    static void led_on(unsigned i) {
        if (i >= 1 && i <= 8) LED_PAD_ON(GPIOG, i);
    }

    /**
     * Turn off indexed green LED on RM Board 2018 A
     * @param i   LED index [1-8]
     */
    static void led_off(unsigned i) {
        if (i >= 1 && i <= 8) LED_PAD_OFF(GPIOG, i);
    }

    /**
     * Toggle indexed green LED on RM Board 2018 A
     * @param i   LED index [1-8]
     */
    static void led_toggle(unsigned i) {
        if (i >= 1 && i <= 8) palTogglePad(GPIOG, i);
    }
#else
    static void led_on(unsigned i) {}

    static void led_off(unsigned i) {}

    static void led_toggle(unsigned i) {}
#endif

    /**
     * Turn off all LEDs
     */
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
// Perform action when ChibiOS halts.
void set_led_when_halt(void);
}

#endif

/** @} */