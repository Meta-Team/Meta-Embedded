//
// Created by liuzikai on 2019-06-25.
//

#ifndef META_INFANTRY_INSPECTOR_H
#define META_INFANTRY_INSPECTOR_H

#include "ch.hpp"

#include "can_interface.h"
#include "ahrs_abstract.h"
#include "remote_interpreter.h"
#include "chassis_interface.h"
#include "gimbal_interface.h"

class Inspector {

public:

    static void init(CANInterface *can1_, AbstractAHRS *ahrs_, tprio_t thread_prio);

    static void startup_check_can();
    static void startup_check_mpu();
    static void startup_check_ist();
    static void startup_check_remote();
    static void startup_check_gimbal_feedback();
    static void startup_check_chassis_feedback();


private:

    static AbstractAHRS *ahrs;
    static CANInterface *can1;

};


#endif //META_INFANTRY_INSPECTOR_H
