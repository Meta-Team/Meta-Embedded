//
// Created by liuzikai on 8/18/21.
//

#ifndef META_EMBEDDED_MOCK_HAL_PAL_H
#define META_EMBEDDED_MOCK_HAL_PAL_H

#define PAL_LOW  0U
#define PAL_HIGH 1U

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

void palSetPad(int port, int pad);
void palClearPad(int port, int pad);
void palTogglePad(int port, int pad);

#endif //META_EMBEDDED_MOCK_HAL_PAL_H
