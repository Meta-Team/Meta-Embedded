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
#elif defined(BOARD_RM_C)
// Blue - PH10, Green - PH11, Red PH12. Low to turn on
#define LED_PAD_ON(port, pad) palSetPad(port, pad)
#define LED_PAD_OFF(port, pad) palClearPad(port, pad)
#else
#error "LED has not been defined for selected board"
#endif


/**
 * @name LED
 * @brief LED Controller
 * @note RM Board 2018 has additional 8 green LEDs
 * @note RM Board C has an additional blue LED
 */
class LED {

public:

    /**
     * Turn off green LED
     */
    static void green_off() {
#if defined(BOARD_RM_C)
        LED_PAD_OFF(GPIOH, GPIOH_LED_G);
#else
        LED_PAD_OFF(GPIOF, GPIOF_LED_GREEN);
#endif
    }

    /**
     * Turn on green LED
     */
    static void green_on() {
#if defined(BOARD_RM_C)
        LED_PAD_ON(GPIOH, GPIOH_LED_G);
#else
        LED_PAD_ON(GPIOF, GPIOF_LED_GREEN);
#endif
    }

    /**
     * Toggle green LED
     */
    static void green_toggle() {
#if defined(BOARD_RM_C)
        palTogglePad(GPIOH, GPIOH_LED_G);
#else
        palTogglePad(GPIOF, GPIOF_LED_GREEN);
#endif
    }

    /**
     * Turn off red LED
     */
    static void red_off() {

#if defined(BOARD_RM_C)
        LED_PAD_OFF(GPIOH, GPIOH_LED_R);
#else
        LED_PAD_OFF(GPIOF, GPIOF_LED_RED);
#endif
    }

    /**
     * Turn on red LED
     */
    static void red_on() {
#if defined(BOARD_RM_C)
        LED_PAD_ON(GPIOH, GPIOH_LED_R);
#else
        LED_PAD_ON(GPIOF, GPIOF_LED_RED);
#endif
    }

    /**
     * Toggle red LED
     */
    static void red_toggle() {
#if defined(BOARD_RM_C)
        palTogglePad(GPIOH, GPIOH_LED_R);
#else
        palTogglePad(GPIOF, GPIOF_LED_RED);
#endif
    }

    /**
     * Turn off blue LED
     */
    static void blue_off() {
#if defined(BOARD_RM_C)
        LED_PAD_OFF(GPIOH, GPIOH_LED_B);
#endif
    }

    /**
     * Turn on blue LED
     */
    static void blue_on() {
#if defined(BOARD_RM_C)
        LED_PAD_ON(GPIOH, GPIOH_LED_B);
#endif
    }

    /**
     * Toggle blue LED
     */
    static void blue_toggle() {
#if defined(BOARD_RM_C)
        palTogglePad(GPIOH, GPIOH_LED_B);
#endif
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
        red_off();
        green_off();
#if defined(BOARD_RM_C)
        blue_off();
#endif
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