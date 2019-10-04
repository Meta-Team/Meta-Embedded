//
// Created by liuzikai on 2019-05-10.
//

#include "state_handler.h"

bool StateHandler::canErrorOccured_ = false;
bool StateHandler::remoteDisconnected_ = false;
#ifdef STATE_HANDLER_ENABLE_GIMBAL
bool StateHandler::gimbalSeriousErrorOccured_ = false;
bool StateHandler::bulletLoaderStuck_ = false;
#endif
#ifdef STATE_HANDLER_ENABLE_CHASSIS
bool StateHandler::chassisSeriousErrorOccured_ = false;
#endif
#ifdef STATE_HANDLER_ENABLE_ELEVATOR
bool StateHandler::elevatorSeriousErrorOccured_ = false;
#endif
#ifdef STATE_HANDLER_ENABLE_ROBOTIC_ARM
bool StateHandler::roboticArmSeriousErrorOccured_ = false;
#endif

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
#ifdef STATE_HANDLER_ENABLE_MPU6500
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
#ifdef STATE_HANDLER_ENABLE_GIMBAL
        case GIMBAL_DISCONNECTED:
            if (!gimbalSeriousErrorOccured_) {
                gimbalSeriousErrorOccured_ = true;
                if (!fromISR) {
                    LOG_ERR("GIMBAL MOTOR %d DISCONNECTED!", va_arg(args, int));
                }
            }
            break;
        case BULLET_LOADER_STUCK:
            if (!bulletLoaderStuck_) {
                bulletLoaderStuck_ = true;
                // Stuck is not a serious error. Just handle it.
            }
            break;
#endif
#ifdef STATE_HANDLER_ENABLE_CHASSIS
        case CHASSIS_DISCONNECTED:
            if (!chassisSeriousErrorOccured_) {
                chassisSeriousErrorOccured_ = true;
                if (!fromISR) {
                    LOG_ERR("CHASSIS MOTOR %d DISCONNECTED!", va_arg(args, int));
                }
            }
            break;
#endif
#ifdef STATE_HANDLER_ENABLE_ELEVATOR
        case ELEVATOR_DISCONNECTED:
            if (!elevatorSeriousErrorOccured_) {
                elevatorSeriousErrorOccured_ = true;
                if (!fromISR) {
                    LOG_ERR("ELEVATOR MOTOR %d DISCONNECTED!", va_arg(args, int));
                }
            }
            break;
        case ELEVATOR_UNBALANCE:
            if (!elevatorSeriousErrorOccured_) {
                elevatorSeriousErrorOccured_ = true;
                if (!fromISR) {
                    LOG_ERR("ELEVATOR UNBALANCE!");
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

            break;
        case CAN_START_SUCCESSFULLY:
            LED::led_on(2);
            break;
#ifdef STATE_HANDLER_ENABLE_MPU6500
        case MPU6500_START_SUCCESSFULLY:
            LED::led_on(3);
            break;
#endif
        case REMOTE_START_SUCCESSFULLY:
            LED::led_on(4);
            break;
#ifdef STATE_HANDLER_ENABLE_GIMBAL
        case GIMBAL_CONNECTED:
            LED::led_on(5);
            break;
#endif
#ifdef STATE_HANDLER_ENABLE_ROBOTIC_ARM
        case ROBOTIC_ARM_CONNECT:
            LED::led_on(5);
            break;
#endif
#ifdef STATE_HANDLER_ENABLE_CHASSIS
        case CHASSIS_CONNECTED:
            LED::led_on(6);
            break;
#endif
#ifdef STATE_HANDLER_ENABLE_ELEVATOR
        case ELEVATOR_CONNECTED:
            LED::led_on(7);
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