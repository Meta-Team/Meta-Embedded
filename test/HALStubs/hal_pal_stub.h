//
// Created by liuzikai on 8/15/21.
//

#ifndef META_EMBEDDED_HAL_PAL_H
#define META_EMBEDDED_HAL_PAL_H

#include <cstdint>

enum GPIO : unsigned {
    GPIOA = 0,
    GPIOB,
    GPIOC,
    GPIOD,
    GPIOE,
    GPIOF,
    GPIOG,
    GPIOH,
    GPIOI,
    GPIOJ,
    GPIOK,
    GPIO_COUNT
};

using stm32_gpio_t = uint32_t;

void palSetPad(unsigned port, unsigned pad);
void palClearPad(unsigned port, unsigned pad);
void palTogglePad(unsigned port, unsigned pad);

extern uint32_t gpioImage[GPIO_COUNT];

#endif //META_EMBEDDED_HAL_PAL_H
