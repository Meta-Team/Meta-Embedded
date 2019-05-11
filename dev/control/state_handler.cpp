//
// Created by liuzikai on 2019-05-10.
//

#include "state_handler.h"

bool StateHandler::canErrorOccured_ = false;
bool StateHandler::remoteDisconnected_ = false;
bool StateHandler::gimbalSeriousErrorOccured_ = false;
bool StateHandler::chassisSeriousErrorOccured_ = false;

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
                LOG_ERR("CAN error: %u", va_arg(args, unsigned));
            }
            break;
#if defined(INFANTRY)
        case MPU6500_DISCONNECTED:
            if (!gimbalSeriousErrorOccured_) {
                gimbalSeriousErrorOccured_ = true;
                if (!fromISR) {
                    LOG_ERR("MPU6500 DISCONNECTED!");
                }
            }
            break;
#endif
        case REMOTE_DISCONNECTED:
            if (!remoteDisconnected_) {
                remoteDisconnected_ = true;
                if (!fromISR) {
                    LOG_ERR("REMOTE DISCONNECTED!");
                }
            }
            break;
#if defined(INFANTRY)
        case GIMBAL_DISCONNECTED:
            if (!gimbalSeriousErrorOccured_) {
                gimbalSeriousErrorOccured_ = true;
                if (!fromISR) {
                    LOG_ERR("GIMBAL MOTOR %d DISCONNECTED!", va_arg(args, int));
                }
            }
            break;
#endif
#if defined(INFANTRY) || defined(ENGINEER)
        case CHASSIS_DISCONNECTED:
            if (!chassisSeriousErrorOccured_) {
                chassisSeriousErrorOccured_ = true;
                if (!fromISR) {
                    LOG_ERR("CHASSIS MOTOR %d DISCONNECTED!", va_arg(args, int));
                }
            }
            break;
#endif

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
        case MAIN_MODULES_SETUP_COMPLETE:
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

#endif