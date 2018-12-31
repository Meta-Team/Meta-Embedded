//
// Created by liuzikai on 2018-12-30.
//

#ifndef META_INFANTRY_GIMBAL_FEEDBACK_MODULE_H
#define META_INFANTRY_GIMBAL_FEEDBACK_MODULE_H

#include "ch.hpp"
#include "hal.h"
#include "serial_shell.h"

class GimbalFeedbackModule : private chibios_rt::BaseStaticThread <512> {

public:

    void start_thread(tprio_t prio) {
        start(prio);
    }

private:

    void main() override;

};


#endif //META_INFANTRY_GIMBAL_FEEDBACK_MODULE_H
