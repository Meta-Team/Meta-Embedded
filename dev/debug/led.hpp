#ifndef _LED_HPP_
#define _LED_HPP_

#define LED_RED_ON palClearPad(GPIOF, 14)
#define LED_RED_OFF palSetPad(GPIOF, 14)
#define LED_RED_TOGGLE palTogglePad(GPIOF, 14)
#define LED_GREEN_ON palClearPad(GPIOE, 7)
#define LED_GREEN_OFF palSetPad(GPIOE, 7)
#define LED_GREEN_TOGGLE palTogglePad(GPIOE, 7)

#endif