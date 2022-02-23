//
// Created by liuzikai on 2018-12-29.
//

/**
 * @file    can_interface.h
 * @brief   CAN interface to receive, distribute and send message.
 *
 * @addtogroup CAN driver
 * @{
 */

#ifndef META_INFANTRY_CAN_INTERFACE_H
#define META_INFANTRY_CAN_INTERFACE_H

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#if defined(BOARD_RM_2018_A)
// CAN1_RX - PD0, CAN1_TX - PD1
#elif defined(BOARD_RM_2017)
// CAN1_RX - PD0, CAN1_TX - PD1
#else
#error "CANInterface has not been defined for selected board"
#endif

#define CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD   TRUE
#define CAN_INTERFACE_THREAD_WORK_AREA_SIZE   1024  // also used in cpp, do not delete

/**
 * CAN interface to receive, distribute and send message
 * @pre CAN pins are configured properly in board.h
 * @usage 1. Create an instance with specific CAN driver
 *        2. Call start() to start the CAN driver and receive thread
 *        3. Register callback functions, waiting for callback
 *        4. Invoke send_msg() to send message through given CAN driver
 */
class CANInterface : public chibios_rt::BaseStaticThread<CAN_INTERFACE_THREAD_WORK_AREA_SIZE> {
public:

    /**
     * Initialize a can interface
     * @param driver   Pointer to CAN driver such as &CAND1
     */
    CANInterface(CANDriver *driver) :
            can_driver(driver), callback_list_count(0)
#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)
            , errorFeedbackThread(driver, last_error_time)
#endif
    {}

    /**
     * Start the CAN driver and the receiving thread
     * @param prio   Thread priority of the receiving thread
     * @return A reference to the created thread
     */
    chibios_rt::ThreadReference start(tprio_t prio);


    /**
     * Type of callback function
     */
    typedef void (*can_callback_func)(CANRxFrame const *rxmsg);

    /**
     * Register a callback. Message with SID in the given range will be distributed to the given callback function
     * @param sid_lower_bound   Minimal SID (inclusive)
     * @param sid_upper_bound   Maximum SID (inclusive)
     * @param callback_func     The function to callback
     * @return Whether registering succeeded or not
     */
    bool register_callback(uint32_t sid_lower_bound, uint32_t sid_upper_bound, can_callback_func callback_func);

    /**
     * Send a frame
     * @param txmsg   The frame to be sent
     * @return Whether the message has been sent successfully
     */
    bool send_msg(const CANTxFrame *txmsg);

    /**
     * Event source to broadcast CAN error message
     */
    EVENTSOURCE_DECL(error_event_src);

private:

    CANDriver *can_driver;

    static constexpr unsigned MAXIMUM_REGISTRATION_COUNT = 20;

    struct callback_resignation_t {
        uint32_t sid_lower_bound;
        uint32_t sid_upper_bound;
        can_callback_func callback_func;
    } callback_list[MAXIMUM_REGISTRATION_COUNT];

    unsigned callback_list_count;


#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)

public:

    time_msecs_t last_error_time = 0;

private:

    class ErrorFeedbackThread : public chibios_rt::BaseStaticThread<512> {
    public:
        ErrorFeedbackThread(CANDriver *can_driver_, time_msecs_t &last_error_time_) :
                can_driver(can_driver_), last_error_time(last_error_time_) {}
        CANDriver *can_driver;
    private:
        time_msecs_t &last_error_time;

        void main() final;
    };

    ErrorFeedbackThread errorFeedbackThread;

#endif


    CANConfig can_cfg = {
            CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
            CAN_BTR_SJW(0) | CAN_BTR_TS2(3) |
            CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
    };
    static constexpr unsigned TRANSMIT_TIMEOUT_MS = 10;

    void main() final;
};


#endif //META_INFANTRY_CAN_INTERFACE_H
