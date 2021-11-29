//
// Created by Chen Qian on 11/17/21.
//

#include "VirtualCOMPort.h"
#include "ch.h"
uint8_t VirtualCOMPort::rxbuffer[100];
uint8_t VirtualCOMPort::txbuffer[100];
uint8_t VirtualCOMPort::rxmode=0;
int16_t VirtualCOMPort::target_torque[2];
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
    chnWriteTimeout(SDU, data,  size, TIME_INFINITE);
}

void VirtualCOMPort::DataReceiveThread::main() {
    setName("vcom_rx_thd");
    while (!shouldTerminate()) {

        chnReadTimeout(SDU, rxbuffer, 5, TIME_INFINITE);

        target_torque[0] = (int16_t)(rxbuffer[1] << 8 | rxbuffer[0]);
        target_torque[1] = (int16_t)(rxbuffer[3] << 8 | rxbuffer[2]);
        rxmode = rxbuffer[4];

        last_update_time = SYSTIME;

    }
}
