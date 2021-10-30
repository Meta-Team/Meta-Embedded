//
// Created by 钱晨 on 10/29/21.
//

#ifndef META_INFANTRY_HAPTIC_SCHEDULER_H
#define META_INFANTRY_HAPTIC_SCHEDULER_H

#include "can_motor_interface.h"
#include "pid_controller.hpp"

using namespace chibios_rt;

class haptic_scheduler {
public:
    static void start(tprio_t SKD_PRIO);
    static PIDController pidController[can_motor_interface::MOTOR_COUNT];
private:
    class skdThread : public BaseStaticThread<512> {
        void main() final;
    };
    static skdThread SKDThread;
};


#endif //META_INFANTRY_HAPTIC_SCHEDULER_H
