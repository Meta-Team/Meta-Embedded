//
// Created by Chen Qian on 11/17/21.
//

#include "VirtualCOMPort.h"

uint8_t VirtualCOMPort::rxbuffer[8];
VirtualCOMPort::DataReceiveThread VirtualCOMPort::data_receive_thd;
time_msecs_t VirtualCOMPort::last_update_time = 0;
float *VirtualCOMPort::torque[2] = {(float *)&VirtualCOMPort::bufferTemp[0], (float *)&VirtualCOMPort::bufferTemp[0]};

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

void VirtualCOMPort::send_angles(float *angles, unsigned int size) {

    chnWriteTimeout(SDU, angles, 4*size, TIME_INFINITE);
}

float VirtualCOMPort::get_torque(unsigned int id) {
    return *torque[id];
}

void VirtualCOMPort::DataReceiveThread::main() {
    setName("vcom_rx_thd");
    while (!shouldTerminate()) {
        chnReadTimeout(SDU, rxbuffer, 8, TIME_INFINITE);

        bufferTemp[0] = (rxbuffer[0] << 24 | rxbuffer[1] << 16 |
                         rxbuffer[2] << 8 | rxbuffer[3]);
        bufferTemp[1] = (rxbuffer[4] << 24 | rxbuffer[5] << 16 |
                         rxbuffer[6] << 8 | rxbuffer[7]);

        last_update_time = SYSTIME;
    }
}
