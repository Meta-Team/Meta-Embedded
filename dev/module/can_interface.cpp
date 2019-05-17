//
// Created by liuzikai on 2018-12-29.
//

#include "can_interface.h"

#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)

void CANInterface::ErrorFeedbackThread::main() {
    setName("can_err_fb");

    event_listener_t el;
    chEvtRegister(&(can_driver->error_event), &el, 0);

    while (!shouldTerminate()) {

        if (waitAnyEventTimeout(ALL_EVENTS, TIME_MS2I(10000)) == 0) {
//            Shell::printf("--- End of error in CAN in last 10 s ---" SHELL_NEWLINE_STR);
            continue;
        }

        eventflags_t flags = chEvtGetAndClearFlags(&el);
        StateHandler::raiseException(StateHandler::CAN_ERROR, flags);
    }

    chEvtUnregister(&(can_driver->rxfull_event), &el);
}

#endif


chibios_rt::ThreadReference CANInterface::start(tprio_t prio) {
    canStart(can_driver, &can_cfg);
#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)
    errorFeedbackThread.can_driver = can_driver;
    errorFeedbackThread.start(LOWPRIO);
#endif
    return chibios_rt::BaseStaticThread<CAN_INTERFACE_THREAD_WORK_AREA_SIZE>::start(prio);
}

bool CANInterface::register_callback(uint32_t sid_lower_bound, uint32_t sid_upper_bound,
                                     CANInterface::can_callback_func callback_func) {
    if (callback_list_count >= maximum_registration_count) return false;
    callback_list[callback_list_count++] = {sid_lower_bound, sid_upper_bound, callback_func};
    return true;
}

void CANInterface::main() {

    if (can_driver == &CAND1) {
        setName("CAN1");
    } else if (can_driver == &CAND2) {
        setName("CAN2");
    } else {
        setName("CAN_Unknown");
    }

    CANRxFrame rxmsg;
    event_listener_t el;

    // Register an event listener for the rx event source from CANInterface driver
    chEvtRegister(&(can_driver->rxfull_event), &el, 0);

    while (!shouldTerminate()) {

        // Wait until a event occurs, or timeout
        if (waitAnyEventTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
            // TODO: maybe here is the place to detect lost of signal
            continue;
        }

        // Process every received message
        while (canReceive(can_driver, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
            chSysLock();
            for (int i = 0; i < callback_list_count; i++) {
                if (rxmsg.SID >= callback_list[i].sid_lower_bound && rxmsg.SID <= callback_list[i].sid_upper_bound) {
                    callback_list[i].callback_func(&rxmsg);
                }
            }
            chSysUnlock();
        }

    }

    chEvtUnregister(&(can_driver->rxfull_event), &el);
}

bool CANInterface::send_msg(const CANTxFrame *txmsg) {
    if (canTransmit(can_driver, CAN_ANY_MAILBOX, txmsg, TIME_MS2I(transmit_timeout_ms)) != MSG_OK) {
        // TODO: show debug info for failure
        return false;
    }
    return true;
}

