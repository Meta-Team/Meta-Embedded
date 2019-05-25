//
// Created by liuzikai on 2018-12-29.
//

#ifndef META_INFANTRY_CAN_H
#define META_INFANTRY_CAN_H

#include "ch.hpp"
#include "hal.h"
#include "led.h"
#include "debug/shell/shell.h"

#include "state_handler.h"

#if defined(BOARD_RM_2018_A)
// CAN1_RX - PD0, CAN1_TX - PD1
#elif defined(BOARD_RM_2017)
// CAN1_RX - PD0, CAN1_TX - PD1
#else
#error "CANInterface has not been defined for selected board"
#endif

#define CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD TRUE
#define CAN_INTERFACE_THREAD_WORK_AREA_SIZE 512

/**
 * @brief CAN driver to receive message and send message
 * @pre CAN pins are configurated properly in board.h
 * @usage 1. create an instance with specific CAN driver
 *        2. call start() to start the CAN driver and receive thread
 *        3. register callback functions
 */
class CANInterface : public chibios_rt::BaseStaticThread<CAN_INTERFACE_THREAD_WORK_AREA_SIZE> {
public:

    /**
     * @brief initialize a can interface
     * @param driver pointer to can driver
     */
    CANInterface(CANDriver *driver) : can_driver(driver), callback_list_count(0) {}

    /**
     * @brief start the CAN driver and the receive thread
     * @param prio
     * @return the same as start() of thread
     */
    chibios_rt::ThreadReference start(tprio_t prio);


    typedef void (*can_callback_func)(CANRxFrame const *rxmsg); // type of callback function

    /**
     * @brief register a callback
     * @param sid_lower_bound minimal SID (inclusive)
     * @param sid_upper_bound maximum SID (inclusive)
     * @param callback_func
     * @return register success or not
     */
    bool register_callback(uint32_t sid_lower_bound, uint32_t sid_upper_bound, can_callback_func callback_func);

    /**
     * @brief send a frame
     * @param txmsg The frame to be sent
     * @return whether the message has been sent successfully
     */
    bool send_msg(const CANTxFrame *txmsg);

private:
#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)

    class ErrorFeedbackThread : public chibios_rt::BaseStaticThread<512> {
    public:
        CANDriver *can_driver;
    private:
        void main() final;
    };

    ErrorFeedbackThread errorFeedbackThread;

#endif

private:

    /** Configurations **/

    CANConfig can_cfg = {
            CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
            CAN_BTR_SJW(0) | CAN_BTR_TS2(3) |
            CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
    };
    static constexpr int transmit_timeout_ms = 10;
    static constexpr int maximum_registration_count = 20;

private:

    CANDriver *can_driver;

    struct callback_resignation_t {
        uint32_t sid_lower_bound;
        uint32_t sid_upper_bound;
        can_callback_func callback_func;
    } callback_list[maximum_registration_count];

    int callback_list_count;

    void main();  // the thread main function
};


#endif //META_INFANTRY_CAN_H
