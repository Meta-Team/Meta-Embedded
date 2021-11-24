//
// Created by Quoke on 11/24/2021.
//

#ifndef META_INFANTRY_COMMUNICATOR_H
#define META_INFANTRY_COMMUNICATOR_H

#include "VirtualCOMPort.h"
#include "haptic_logic.h"

using namespace chibios_rt;
class Communicator {

    static void init(tprio_t communicator_prio_);

    class CommunicatorSKD : public BaseStaticThread<512> {
        void main() final;
    };

    static CommunicatorSKD communicator_skd;
};


#endif //META_INFANTRY_COMMUNICATOR_H
