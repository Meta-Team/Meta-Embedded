//
// Created by liuzikai on 2019-05-10.
//

#ifndef META_INFANTRY_ERROR_HANDLER_H
#define META_INFANTRY_ERROR_HANDLER_H

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"
#include "buzzer_scheduler.h"

/**
 * @name StateHandler
 * @brief This module is a collection of operations for state and error handling. These code pieces are collected here
 *        to allow flexible feedback to user.
 * @note This module should only set upon debug module, but NOT any higher level module. Events or Exceptions should be
 *       be reported by threads (although may through module code), instead of being fetched by this module.
 */

#if defined(INFANTRY) || defined(HERO)
#define STATE_HANDLER_ENABLE_MPU6500
#endif

#if defined(INFANTRY) || defined(HERO)
#define STATE_HANDLER_ENABLE_GIMBAL
#endif

#if defined(INFANTRY) || defined(ENGINEER) || defined(HERO)
#define STATE_HANDLER_ENABLE_CHASSIS
#endif

#if defined(ENGINEER)
#define STATE_HANDLER_ENABLE_ROBOTIC_ARM
#define STATE_HANDLER_ENABLE_ELEVATOR
#endif

class StateHandler {

public:

    /**
     * Events collection
     */
    enum Events {
        SHELL_START
        ,CAN_START_SUCCESSFULLY
#ifdef STATE_HANDLER_ENABLE_MPU6500
        ,MPU6500_START_SUCCESSFULLY
#endif
        ,REMOTE_START_SUCCESSFULLY
#ifdef STATE_HANDLER_ENABLE_GIMBAL
        ,GIMBAL_CONNECTED
#endif
#ifdef STATE_HANDLER_ENABLE_CHASSIS
        ,CHASSIS_CONNECTED
#endif
#ifdef STATE_HANDLER_ENABLE_ROBOTIC_ARM
        ,ROBOTIC_ARM_CONNECT
#endif
#ifdef STATE_HANDLER_ENABLE_ELEVATOR
        ,ELEVATOR_CONNECTED
#endif
        ,MAIN_MODULES_SETUP_COMPLETE
        ,MAIN_THREAD_SETUP_COMPLETE
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
        CAN_ERROR
#ifdef STATE_HANDLER_ENABLE_MPU6500
        ,MPU6500_DISCONNECTED
#endif
        ,REMOTE_DISCONNECTED
#ifdef STATE_HANDLER_ENABLE_GIMBAL
        ,GIMBAL_DISCONNECTED
        ,BULLET_LOADER_STUCK
#endif
#if defined(HERO)
        ,BULLET_PLATE_STUCK
#endif
#ifdef STATE_HANDLER_ENABLE_CHASSIS
        ,CHASSIS_DISCONNECTED
#endif
#ifdef STATE_HANDLER_ENABLE_ELEVATOR
        ,ELEVATOR_DISCONNECTED
        ,ELEVATOR_UNBALANCE
#endif
#ifdef STATE_HANDLER_ENABLE_ROBOTIC_ARM
        ,ROBOTIC_ARM_DISCONNECTED
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
    static bool fetchCANErrorMark() {
        bool ret = canErrorOccured_;
        canErrorOccured_ = false;
        return ret;
    }

    static bool remoteDisconnected() { return remoteDisconnected_; }

#ifdef STATE_HANDLER_ENABLE_GIMBAL
    static bool gimbalSeriousErrorOccured() { return gimbalSeriousErrorOccured_; }
    static bool bulletLoaderStuck() { return bulletLoaderStuck_; }
    static void bulletLoaderSmooth() { bulletLoaderStuck_ = false; }
#endif
#if defined(HERO)
    static bool bulletPlateStuck() { return  bulletPlateStuck_; }
#endif
#ifdef STATE_HANDLER_ENABLE_CHASSIS
    static bool chassisSeriousErrorOccured() { return chassisSeriousErrorOccured_; }
#endif
#ifdef STATE_HANDLER_ENABLE_ELEVATOR
    static bool elevatorSeriousErrorOccured() { return elevatorSeriousErrorOccured_; };
#endif
#ifdef STATE_HANDLER_ENABLE_ROBOTIC_ARM
    static bool roboticArmSeriousErrorOccured() { return roboticArmSeriousErrorOccured_; }
#endif

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
#ifdef STATE_HANDLER_ENABLE_GIMBAL
    static bool gimbalSeriousErrorOccured_;
    static bool bulletLoaderStuck_;
#endif
#ifdef HERO
    static bool bulletPlateStuck_;
#endif
#ifdef STATE_HANDLER_ENABLE_CHASSIS
    static bool chassisSeriousErrorOccured_;
#endif
#ifdef STATE_HANDLER_ENABLE_ELEVATOR
    static bool elevatorSeriousErrorOccured_;
#endif
#ifdef STATE_HANDLER_ENABLE_ROBOTIC_ARM
    static bool roboticArmSeriousErrorOccured_;
#endif

};


#endif //META_INFANTRY_ERROR_HANDLER_H
