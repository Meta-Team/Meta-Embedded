//
// Created by Chen Qian on 11/17/21.
//

#include "VirtualCOMPort.h"

uint8_t VirtualCOMPort::buffer[100];
VirtualCOMPort::DataReceiveThread VirtualCOMPort::data_receive_thd;
time_msecs_t VirtualCOMPort::last_update_time = 0;

void VirtualCOMPort::init(SerialUSBDriver *SDU_, tprio_t rx_thd_prio) {
    SDU = SDU_;
    sduObjectInit(SDU);
    sduStart(SDU, &serusbcfg);

    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    data_receive_thd.start(rx_thd_prio);
}

void VirtualCOMPort::send_data(uint8_t *data, unsigned int size) {
    chnWriteTimeout(SDU, data, size, TIME_INFINITE);
}

void VirtualCOMPort::DataReceiveThread::main() {
    setName("vcom_rx_thd");
    while (!shouldTerminate()) {
        chnReadTimeout(SDU, buffer, sizeof(buffer), TIME_INFINITE);
        last_update_time = SYSTIME;
    }
}
