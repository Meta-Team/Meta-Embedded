#ifndef _LED_HPP_
#define _LED_HPP_

#define LED_D2_ON palClearPad(GPIOA, 6)
#define LED_D2_OFF palSetPad(GPIOA, 6)
#define LED_D3_ON palClearPad(GPIOA, 7)
#define LED_D3_OFF palSetPad(GPIOA, 7)

#endif