//
// Created by liuzikai on 2019-06-25.
//

#ifndef META_INFANTRY_INSPECTOR_INFANTRY_H
#define META_INFANTRY_INSPECTOR_INFANTRY_H

#include "ch.hpp"

#include "buzzer_scheduler.h"

#include "can_interface.h"
#include "ahrs_abstract.h"
#include "remote_interpreter.h"
#include "chassis_interface.h"
#include "gimbal_interface.h"

#if defined(INFANTRY)
#include "vehicle_infantry.h"
#else
#error "Files inspector_infantry.h/cpp should only be used for Infantry main program"
#endif

class InspectorI {

public:

    static void init(CANInterface *can1_, CANInterface *can2_, AbstractAHRS *ahrs_);

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
    static CANInterface *can2;

    static bool gimbal_failure_;
    static bool chassis_failure_;
    static bool remote_failure_;

    static bool check_gimbal_failure();
    static bool check_chassis_failure();
    static bool check_remote_data_error();

    /// Inspector Thread
    class InspectorThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned INSPECTOR_THREAD_INTERVAL = 20;  // [ms]
        void main();
    };

    static InspectorThread inspectorThread;

};


#endif //META_INFANTRY_INSPECTOR_INFANTRY_H
