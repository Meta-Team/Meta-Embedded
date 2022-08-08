//
// Created by liuzikai on 2019-06-25.
//

#ifndef META_SENTRY_INSPECTOR_H
#define META_SENTRY_INSPECTOR_H

#include "ch.hpp"
#include "hardware_conf.h"

#include "buzzer_scheduler.h"

#include "can_interface.h"
#include "can_motor_interface.h"

#if ENABLE_AHRS
#include "ahrs_abstract.h"
#endif

#include "remote_interpreter.h"
#include "vehicle_sentry.h"

#if defined(SENTRY)
#include "vehicle_sentry.h"
#else
#error "Files inspector_sentry.h/cpp should only be used for Sentry main program"
#endif

class InspectorS {

public:

    static void init(CANInterface *can1_, CANInterface *can2_
#if ENABLE_AHRS
                     , AbstractAHRS *ahrs_);

    static void startup_check_mpu();
    static void startup_check_ist();
#else
    );
#endif

    static void start_inspection(tprio_t thread_prio);

    static void startup_check_can();
    static void startup_check_remote();
    static void startup_check_gimbal_feedback();
    static void startup_check_chassis_feedback();

    static bool gimbal_failure();
    static bool chassis_failure();
    static bool remote_failure();

private:
#if ENABLE_AHRS
    static AbstractAHRS *ahrs;
#endif
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


#endif //META_SENTRY_INSPECTOR_H
