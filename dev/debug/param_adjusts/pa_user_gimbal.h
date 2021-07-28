//
// Created by liuzikai on 7/29/21.
//

#ifndef META_INFANTRY_PA_USER_GIMBAL_H
#define META_INFANTRY_PA_USER_GIMBAL_H

#include "gimbal_scheduler.h"
#include "remote_interpreter.h"

class PAUserGimbal : protected chibios_rt::BaseStaticThread<512> {
public:

    chibios_rt::ThreadReference start(tprio_t prio);

private:

    void main() override;

};


#endif //META_INFANTRY_PA_USER_GIMBAL_H
