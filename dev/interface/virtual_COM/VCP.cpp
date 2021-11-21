//
// Created by Chen Qian on 11/17/21.
//

#include "VCP.h"

void VCP::init(SerialUSBDriver *SDU_) {
    SDU = SDU_;
    sduObjectInit(SDU);
    sduStart(SDU, &serusbcfg);

    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
}

void VCP::transmitTest() {
    chnWriteTimeout(SDU, reinterpret_cast<const uint8_t *>("Hello World!"), 12, TIME_INFINITE);
}

