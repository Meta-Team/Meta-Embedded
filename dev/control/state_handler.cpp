//
// Created by liuzikai on 2019-05-10.
//

#include "state_handler.h"

bool StateHandler::canErrorOccured_ = false;

#ifdef ENABLE_STATE_HANDLE

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
            if (!fromISR) {
                Shell::printf("CAN error: %u" SHELL_NEWLINE_STR, va_arg(args, unsigned));
            }
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
#if defined(INFANTRY)
        case MPU6500_START_SUCCESSFULLY:
            LED::led_on(3);
            break;
#endif
        case REMOTE_START_SUCCESSFULLY:
            LED::led_on(4);
            break;
#if defined(INFANTRY)
        case GIMBAL_CONNECTED:
            LED::led_on(5);
            break;
#endif
#if defined(INFANTRY) || defined(ENGINEER)
        case CHASSIS_CONNECTED:
            LED::led_on(6);
            break;
#endif
        case MAIN_MODULES_SETUP_COMMPLETE:
            LED::green_on();
            break;
        case MAIN_THREAD_SETUP_COMPLETE:
            Buzzer::play_sound(Buzzer::sound_startup_intel, LOWPRIO);
            break;
    }
}

bool StateHandler::fetchCANErrorMark() {
    bool ret = canErrorOccured_;
    canErrorOccured_ = false;
    return ret;
}

#else  // NULL implementation
void StateHandler::echoEvent(StateHandler::Events event, ...) {
    (void) event;
}

void StateHandler::echoEventFromISR(StateHandler::Events event, ...) {
    (void) event;
}

void StateHandler::raiseException(StateHandler::Exceptions exception, ...) {
    (void) exception;
}

void StateHandler::raiseExceptionFromISR(StateHandler::Exceptions exception, ...) {
    (void) exception;
}

void StateHandler::handleException(StateHandler::Exceptions exception, bool fromISR, ...) {
    (void) exception;
    (void) fromISR;
}

void StateHandler::handleEvent(StateHandler::Events event, bool fromISR, ...) {
    (void) event;
    (void) fromISR;
}

bool StateHandler::performAction(StateHandler::Actions action) {
    (void) action;
    return false;
}
#endif