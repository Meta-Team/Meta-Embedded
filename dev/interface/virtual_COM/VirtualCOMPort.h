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

    static uint8_t rxbuffer[8];

    static time_msecs_t last_update_time;

    static float* torque[2];

    static uint32_t bufferTemp[2];

    static float get_torque(unsigned int id);

    static void send_angles(float *angles, unsigned int size);

    class DataReceiveThread : public BaseStaticThread<512> {
        void main() final;
    };

    static DataReceiveThread data_receive_thd;
};

#endif //META_INFANTRY_VIRTUALCOMPORT_H
