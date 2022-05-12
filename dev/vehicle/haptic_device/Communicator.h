//
// Created by Quoke on 11/24/2021.
//

#ifndef META_INFANTRY_COMMUNICATOR_H
#define META_INFANTRY_COMMUNICATOR_H

#include "usb_serial_interface.h"
#include "CANBUS_MOTOR_CFG.h"
#include "haptic_logic.h"

using namespace chibios_rt;
class Communicator {
public:

    /**
     * @brief Initiate the communicator.
     */
    static void init(tprio_t communicator_prio_, tprio_t rx_prio_);

    /**
     * @brief send_angles
     */
    static uint8_t tx_angles[CANMotorCFG::MOTOR_COUNT*2+1];

    static int16_t target_torque[CANMotorCFG::MOTOR_COUNT];

    static uint8_t rxmode;

    static time_msecs_t last_update_time;

    static void send_data(uint8_t *data, unsigned int size);
private:

    /**
     * @brief Transmit package size through VCP
     */
    static const int pak_size = 5;

    /**
     * @brief VCP buffer.
     */
    static uint8_t buffer[pak_size];

    /**
     * @brief Rx Thread for CDC
     */
    class RxThread : public BaseStaticThread<512> {
        void main() final;
    };

    static RxThread rx_thd;
    /**
     * @brief TxRxThread for CDC
     */
    class CommunicatorThd : public BaseStaticThread<512> {
        void main() final;
    };

    static CommunicatorThd communicator_thd;

};


#endif //META_INFANTRY_COMMUNICATOR_H
