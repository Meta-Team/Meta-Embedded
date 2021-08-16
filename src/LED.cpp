//
// Created by liuzikai on 8/15/21.
//

#include "LED.h"
#include "hal.h"

namespace LED {

void redOnX() {
    SET_LED_ON(LED_RED_GPIO_PORT, LED_RED_GPIO_PAD);
}

void redOffX() {
    SET_LED_OFF(LED_RED_GPIO_PORT, LED_RED_GPIO_PAD);
}

void redToggleX() {
    palTogglePad(LED_RED_GPIO_PORT, LED_RED_GPIO_PAD);
}

void greenOnX() {
    SET_LED_ON(LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PAD);
}

void greenOffX() {
    SET_LED_OFF(LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PAD);
}

void greenToggleX() {
    palTogglePad(LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PAD);
}

static inline bool numberIsOutOfBound(int number) {
    return (number < NUMBERED_LED_MIN || number > NUMBERED_LED_MAX);
}

void numberOnX(int number) {
    if (numberIsOutOfBound(number)) return;
    SET_LED_ON(NUMBERED_LED_GPIO_PORT, NUMBERED_LED_GPIO_PAD(number));
}

void numberOffX(int number) {
    if (numberIsOutOfBound(number)) return;
    SET_LED_OFF(NUMBERED_LED_GPIO_PORT, NUMBERED_LED_GPIO_PAD(number));
}

void numberToggleX(int number) {
    if (numberIsOutOfBound(number)) return;
    palTogglePad(NUMBERED_LED_GPIO_PORT, NUMBERED_LED_GPIO_PAD(number));
}

void allOffX() {
    redOffX();
    greenOffX();
    for (int i = NUMBERED_LED_MIN; i <= NUMBERED_LED_MAX; i++) numberOffX(i);
}

}