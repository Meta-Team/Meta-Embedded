//
// Created by liuzikai on 2019-05-10.
//

#ifndef META_INFANTRY_ERROR_HANDLER_H
#define META_INFANTRY_ERROR_HANDLER_H

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"

class StateHandler {

public:

    enum Events {
        SHELL_START,
        CAN_START_SUCCESSFULLY,
        REMOTE_START,
        MPU6500_START
    };

    static void echoEvent(Events event, ...);

    static void echoEventFromISR(Events event, ...);

    enum Exceptions {
        REMOTE_START_UART_FAILED,
        REMOTE_START_RECEIVE_FAILED,
        CAN_ERROR
    };

    static void raiseException(Exceptions exception, ...);

    static void raiseExceptionFromISR(Exceptions exception, ...);

    enum Actions {
        CAN_CHECK_NO_ERROR
    };

    static bool performAction(Actions action);

private:

    static void handleEvent(Events event, bool fromISR, ...);

    static void handleException(Exceptions exception, bool fromISR, ...);

    static bool canErrorOccured_;

};


#endif //META_INFANTRY_ERROR_HANDLER_H
