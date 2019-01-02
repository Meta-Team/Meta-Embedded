//
// Created by liuzikai on 2018-12-29.
//

#include "can_interface.h"

void CANInterface::start_can() {
    canStart(can_driver, &can_cfg);
}

void CANInterface::main() {

    if (can_driver == &CAND1) {
        setName("CAN1_Thread");
    } else {
        setName("CAN_Unknown_Thread");
    }

    CANRxFrame rxmsg;
    event_listener_t el;

    // Register an event listener for the rx event source from CANInterface driver
    chEvtRegister(&(can_driver->rxfull_event), &el, 0);

    while (!shouldTerminate()) {

        // Wait until a event occurs, or timeout
        if (waitAnyEventTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
            // TODO: maybe here is the way to detect lost of signal
            continue;
        }

        // Process every received message
        while (canReceive(can_driver, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
            chSysLock();
            rx_callback (&rxmsg);
            chSysUnlock();
        }

    }
    chEvtUnregister(&(can_driver->rxfull_event), &el);
}

bool CANInterface::send_msg(const CANTxFrame *txmsg) {
    if(canTransmit(can_driver, CAN_ANY_MAILBOX, txmsg, TIME_MS2I(transmit_timeout_ms)) != MSG_OK) {
        // TODO: show debug info for failure
        return false;
    }
    return true;
}