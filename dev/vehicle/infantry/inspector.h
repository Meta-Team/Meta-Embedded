//
// Created by liuzikai on 2019-06-25.
//

#ifndef META_INFANTRY_INSPECTOR_H
#define META_INFANTRY_INSPECTOR_H

#include "ch.hpp"

#include "buzzer.h"

#include "can_interface.h"
#include "ahrs_abstract.h"
#include "remote_interpreter.h"
#include "chassis_interface.h"
#include "gimbal_interface.h"

class Inspector {

public:

    static void init(CANInterface *can1_, AbstractAHRS *ahrs_);

    static void start_inspection(tprio_t thread_prio);

    static void startup_check_can();
    static void startup_check_mpu();
    static void startup_check_ist();
    static void startup_check_remote();
    static void startup_check_gimbal_feedback();
    static void startup_check_chassis_feedback();

    static bool gimbal_failure();
    static bool chassis_failure();
    static bool remote_failure();

private:

    static AbstractAHRS *ahrs;
    static CANInterface *can1;

    static bool gimbal_failure_;
    static bool chassis_failure_;
    static bool remote_failure_;

    static bool check_gimbal_failure();
    static bool check_chassis_failure();

    static constexpr unsigned INSPECTOR_THREAD_INTERVAL = 20;  // [ms]

    class InspectorThread : public chibios_rt::BaseStaticThread<512> {
        void main();
    };

    static InspectorThread inspectorThread;

};


#endif //META_INFANTRY_INSPECTOR_H
