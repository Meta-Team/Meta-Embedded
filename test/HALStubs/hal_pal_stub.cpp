//
// Created by liuzikai on 8/15/21.
//

#include <stdexcept>
#include "hal_pal_stub.h"

uint32_t gpioImage[GPIO_COUNT] = {0};

static constexpr unsigned PAD_COUNT = 32;

static inline void checkValidPortAndPad(unsigned int port, unsigned int pad) {
    if (port >= GPIO_COUNT) throw std::range_error("port out-of-bound");
    if (pad >= PAD_COUNT) throw std::range_error("pad out-of-bound");
}

static inline uint32_t padToPortMask(unsigned int pad) {
    return ((uint32_t) 1U << pad);
}

void palSetPad(unsigned int port, unsigned int pad) {
    checkValidPortAndPad(port, pad);
    gpioImage[port] |= padToPortMask(pad);
}

void palClearPad(unsigned int port, unsigned int pad) {
    checkValidPortAndPad(port, pad);
    gpioImage[port] &= ~padToPortMask(pad);
}

void palTogglePad(unsigned int port, unsigned int pad) {
    checkValidPortAndPad(port, pad);
    gpioImage[port] ^= padToPortMask(pad);
}
