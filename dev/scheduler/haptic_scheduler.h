//
// Created by 钱晨 on 10/29/21.
//

#ifndef META_INFANTRY_HAPTIC_SCHEDULER_H
#define META_INFANTRY_HAPTIC_SCHEDULER_H

#include "can_motor_interface.h"

using namespace chibios_rt;

class haptic_scheduler {
public:
    void start(tprio_t SKD_PRIO);

private:
    class skdThread : BaseStaticThread<512> {
        void main() final;
    };
};


#endif //META_INFANTRY_HAPTIC_SCHEDULER_H
