//
// Created by Quoke on 11/24/2021.
//

#ifndef META_INFANTRY_COMMUNICATOR_H
#define META_INFANTRY_COMMUNICATOR_H

#include "VirtualCOMPort.h"
#include "haptic_logic.h"

using namespace chibios_rt;
class Communicator {
public:

    /**
     * @brief Initiate the communicator.
     */
    static void init(tprio_t communicator_prio_);

    /**
     * @brief send_angles
     */
    static uint8_t tx_angles[CANMotorCFG::MOTOR_COUNT*2+1];

    /**
     * @brief TxRxThread for CDC
     */
    class CommunicatorThd : public BaseStaticThread<512> {
        void main() final;
    };

    static CommunicatorThd communicator_thd;
};


#endif //META_INFANTRY_COMMUNICATOR_H
