//
// Created by liuzikai on 2018-12-29.
//

#ifndef META_INFANTRY_CAN_H
#define META_INFANTRY_CAN_H

#include "ch.hpp"
#include "hal.h"
#include "led.h"
#include "serial_shell.h"

class CANInterface : public chibios_rt::BaseStaticThread <256> {
public:

    typedef void (*can_callback_func) (CANRxFrame *rxmsg); // type of callback function

    void start_can();  // start CANInterface driver
    void start_thread(tprio_t prio) {  // wrapper function to start the receive thread
        start(prio);
    }

    /**
     * @brief initialize a can interface
     * @param driver              pointer to can driver
     * @param rx_callback_func    pointer to callback function when a message is reveived
     */
    CANInterface(CANDriver* driver, can_callback_func rx_callback_func) {
        can_driver = driver;
        rx_callback = rx_callback_func;
    }

    /**
     * @brief send a frame
     * @param txmsg               The frame to be sent
     * @return whether the message has been sent successfully
     */
    bool send_msg(const CANTxFrame *txmsg);

protected:
    void main() override;  // the thread main function

private:

    /** CAN configurations **/
    CANConfig can_cfg = {
            CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
            CAN_BTR_SJW(0) | CAN_BTR_TS2(3) |
            CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
    };
    static constexpr int transmit_timeout_ms = 10;

    // They will be initialized by instructor
    CANDriver* can_driver;
    can_callback_func rx_callback;
};


#endif //META_INFANTRY_CAN_H
