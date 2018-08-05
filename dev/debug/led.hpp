#ifndef _LED_HPP_
#define _LED_HPP_

#include "ch.hpp"
#include "hal.h"

#define LED_RED_ON palSetPad(GPIOF, 14)
#define LED_RED_OFF palClearPad(GPIOF, 14)
#define LED_RED_TOGGLE palTogglePad(GPIOF, 14)
#define LED_GREEN_ON palSetPad(GPIOE, 7)
#define LED_GREEN_OFF palClearPad(GPIOE, 7)
#define LED_GREEN_TOGGLE palTogglePad(GPIOE, 7)

extern "C" {
#include "chconf.h"
// LED_RED_ON and LED_GREEN_OFF when halt.
void set_led_when_halt(void);
}

#endif