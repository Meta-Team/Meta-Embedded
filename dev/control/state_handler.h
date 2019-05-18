//
// Created by liuzikai on 2019-05-10.
//

#ifndef META_INFANTRY_ERROR_HANDLER_H
#define META_INFANTRY_ERROR_HANDLER_H

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "buzzer.h"

/**
 * @name StateHandler
 * @brief This module is a collection of operations for state and error handling. These code pieces are collected here
 *        to allow flexible feedback to user.
 * @note This module should only set upon debug module, but NOT any higher level module. Events or Exceptions should be
 *       be reported by threads (although may through module code), instead of being fetched by this module.
 */
class StateHandler {

public:

    /**
     * Events collection
     */
    enum Events {
        SHELL_START,
        CAN_START_SUCCESSFULLY,
#if defined(INFANTRY) || defined(HERO)
        MPU6500_START_SUCCESSFULLY,
#endif
        REMOTE_START_SUCCESSFULLY,
#if defined(INFANTRY) || defined(HERO)
        GIMBAL_CONNECTED,
#endif
#if defined(INFANTRY) || defined(ENGINEER) || defined(HERO)
        CHASSIS_CONNECTED,
#endif
        MAIN_MODULES_SETUP_COMPLETE,
        MAIN_THREAD_SETUP_COMPLETE
    };

    /**
     * Perform actions when an event occurs
     * @param event  Event type
     * @param ...    Other data. Developer designed how to process them at the same time they use this method.
     */
    static void echoEvent(Events event, ...);

    /**
     * Perform actions (only those allowed in ISR) when an event occurs
     * @param event   Event type
     * @param ...     Other data. Developer designed how to process them at the same time they use this method.
     */
    static void echoEventFromISR(Events event, ...);

    /**
     * Exceptions collection
     */
    enum Exceptions {
        CAN_ERROR,
#if defined(INFANTRY) || defined(HERO)
        MPU6500_DISCONNECTED,
#endif
        REMOTE_DISCONNECTED,
#if defined(INFANTRY) || defined(HERO)
        GIMBAL_DISCONNECTED,
        BULLET_LOADER_STUCK,
#endif

#if defined(INFANTRY) || defined(ENGINEER) || defined(HERO)
        CHASSIS_DISCONNECTED
#endif
    };

    /**
     * Perform actions when an exception occurs
     * @param exception Exception type
     * @param ...       Other data. Developer designed how to process them at the same time they use this method.
     */
    static void raiseException(Exceptions exception, ...);

    /**
     * Perform actions (only those allowed in ISR) when an exception occurs
     * @param exception Exception type
     * @param ...       Other data. Developer designed how to process them at the same time they use this method.
     */
    static void raiseExceptionFromISR(Exceptions exception, ...);


#if defined(ENABLE_STATE_HANDLE)

    /** Error Specific Methods */

    /**
     * Return the mark of CAN error (set by CANInterface) and clear it
     * @return
     */
    static bool fetchCANErrorMark();

    static bool remoteDisconnected() { return remoteDisconnected_; }

    static bool gimbalSeriousErrorOccured() { return gimbalSeriousErrorOccured_; }

    static bool chassisSeriousErrorOccured() { return chassisSeriousErrorOccured_; }

#endif

private:

    /**
     * Helper function to handle event
     * @param event
     * @param fromISR
     * @param ...
     */
    static void handleEvent(Events event, bool fromISR, ...);

    /**
     * Helper function to handle exception
     * @param exception
     * @param fromISR
     * @param ...
     */
    static void handleException(Exceptions exception, bool fromISR, ...);

    static bool canErrorOccured_;
    static bool remoteDisconnected_;
    static bool gimbalSeriousErrorOccured_;
    static bool chassisSeriousErrorOccured_;

};


#endif //META_INFANTRY_ERROR_HANDLER_H
