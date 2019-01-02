//
// Created by liuzikai on 2018/7/16.
//

#include "led.h"

extern "C" {
void set_led_when_halt(void) {
    LED_RED_ON;
    LED_GREEN_OFF;
}
}