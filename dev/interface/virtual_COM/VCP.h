//
// Created by Chen Qian on 11/17/21.
//

#ifndef META_INFANTRY_VCP_H
#define META_INFANTRY_VCP_H

#include "usbconf.h"

using namespace chibios_rt;
/**
 * @brief Virtual COM port for STM32.
 * @usage Please first define "BOARD_OTG_NOVBUSSENS" in board.h, to establish the appropriate communication with PC.
 * @code
 * 1. Enable HAL_USE_USB in halconf.h
 * 2. Call init function to initialize the VCP.
 * TODO: Sending function and data struct. Or add a property that using shell over VCP.
 * @endcode
 * */
class VCP : public usbconf {
public:
    static void init(SerialUSBDriver *SDU_, tprio_t rx_thd_prio);

    static uint8_t buffer[100];

    static void sendData(uint8_t data[], unsigned int size);

    class DataReceiveThread : public BaseStaticThread<512> {
        void main() final;
    };

    static DataReceiveThread dataReceiveThread;
};

#endif //META_INFANTRY_VCP_H
