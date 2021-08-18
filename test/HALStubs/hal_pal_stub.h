//
// Created by liuzikai on 8/15/21.
//

#ifndef META_EMBEDDED_HAL_PAL_H
#define META_EMBEDDED_HAL_PAL_H

#include <cstdint>

#define PAL_LOW                         0U
#define PAL_HIGH                        1U

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

void palSetPad(unsigned port, unsigned pad);
void palClearPad(unsigned port, unsigned pad);
void palTogglePad(unsigned port, unsigned pad);

using PortImageType = uint16_t;

extern PortImageType gpioPortImage[GPIO_COUNT];

#endif //META_EMBEDDED_HAL_PAL_H
