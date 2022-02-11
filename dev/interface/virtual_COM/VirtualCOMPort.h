//
// Created by Chen Qian on 11/17/21.
//

#ifndef META_INFANTRY_VIRTUALCOMPORT_H
#define META_INFANTRY_VIRTUALCOMPORT_H

#include "usbconf.h"

using namespace chibios_rt;
/**
 * @brief Virtual COM port for STM32.
 * @usage Please first define "BOARD_OTG_NOVBUSSENS" in board.h, to establish the appropriate communication with PC.
 * @code
 * 1. Enable HAL_USE_USB in halconf.h
 * 2. Call init function to initialize the VirtualCOMPort.
 * @endcode
 * */
class VirtualCOMPort : public usbconf {
public:
    static void init(SerialUSBDriver *SDU_, tprio_t rx_thd_prio);

    static uint8_t rxbuffer[100];

    static uint8_t txbuffer[100];

    static uint8_t rxmode;

    static time_msecs_t last_update_time;

    static int16_t target_torque[2];

    static void send_data(uint8_t *data, unsigned int size);

    class DataReceiveThread : public BaseStaticThread<512> {
        void main() final;
    };

    static DataReceiveThread data_receive_thd;
};

#endif //META_INFANTRY_VIRTUALCOMPORT_H
