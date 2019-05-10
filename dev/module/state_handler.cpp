//
// Created by liuzikai on 2019-05-10.
//

#include "state_handler.h"

bool StateHandler::canErrorOccured_ = false;

void StateHandler::echoEvent(StateHandler::Events event, ...) {
    va_list args;
    va_start (args, event);
    handleEvent(event, false, args);
}

void StateHandler::echoEventFromISR(StateHandler::Events event, ...) {
    va_list args;
    va_start (args, event);
    handleEvent(event, true, args);
}

void StateHandler::raiseException(StateHandler::Exceptions exception, ...) {
    va_list args;
    va_start (args, exception);
    handleException(exception, false, args);
}

void StateHandler::raiseExceptionFromISR(StateHandler::Exceptions exception, ...) {
    va_list args;
    va_start (args, exception);
    handleException(exception, true, args);
}

void StateHandler::handleException(StateHandler::Exceptions exception, bool fromISR, ...) {
    va_list args;
    va_start (args, fromISR);

    switch (exception) {
        case CAN_ERROR:
            canErrorOccured_ = true;
            break;
    }
}

void StateHandler::handleEvent(StateHandler::Events event, bool fromISR, ...) {
    va_list args;
    va_start (args, fromISR);

    switch (event) {
        case SHELL_START:
            LED::led_on(1);
            break;
        case CAN_START_SUCCESSFULLY:
            LED::led_on(2);
            break;
        case REMOTE_START:
            LED::led_on(3);
            break;
        case MPU6500_START:
            LED::led_on(4);
            break;
    }
}

bool StateHandler::performAction(StateHandler::Actions action) {
    switch (action) {
        case CAN_CHECK_NO_ERROR:  // wait for 500ms during which no CAN error occurs
            time_msecs_t t = SYSTIME;
            while (SYSTIME - t > 100) {
                if (canErrorOccured_) {
                    t = SYSTIME;  // reset the counter
                    canErrorOccured_ = false;  // clear the state mark
                }
                chThdSleepMilliseconds(5);
            }
            handleEvent(CAN_START_SUCCESSFULLY, false);
            break;
    }
}