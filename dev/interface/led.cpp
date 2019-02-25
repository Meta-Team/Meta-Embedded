//
// Created by liuzikai on 2018/7/16.
// NOTICE: this file is needed to avoid multiple definitions of set_led_when_halt()
//

#include "led.h"

extern "C" {
void set_led_when_halt(void) {
    palClearPad(GPIOF, GPIOF_LED_GREEN); // LED_RED_ON
    palSetPad(GPIOF, GPIOF_LED_GREEN); // LED_GREEN_OFF
}
}