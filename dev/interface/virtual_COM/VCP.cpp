//
// Created by Chen Qian on 11/17/21.
//

#include "VCP.h"

uint8_t VCP::buffer[100];
VCP::DataReceiveThread VCP::dataReceiveThread;

void VCP::init(SerialUSBDriver *SDU_, tprio_t rx_thd_prio) {
    SDU = SDU_;
    sduObjectInit(SDU);
    sduStart(SDU, &serusbcfg);

    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    dataReceiveThread.start(rx_thd_prio);
}

void VCP::sendData(uint8_t *data, unsigned int size) {
    chnWriteTimeout(SDU, data, size, TIME_INFINITE);
}

void VCP::DataReceiveThread::main() {
    setName("vcom_rx_thd");
    while (!shouldTerminate()) {
        chnReadTimeout(SDU, buffer, sizeof(buffer), TIME_INFINITE);
    }
}
