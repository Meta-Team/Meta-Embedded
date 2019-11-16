//
// Created by Kerui Zhu on 7/19/2019.
//

#ifndef META_INFANTRY_INSPECTOR_AERIAL_H
#define META_INFANTRY_INSPECTOR_AERIAL_H

#include "ch.hpp"

#include "buzzer_scheduler.h"

#include "can_interface.h"
#include "ahrs_abstract.h"
#include "remote_interpreter.h"
#include "gimbal_interface.h"
#include "referee_interface.h"

#include "vehicle_aerial.h"

class InspectorA {

public:

    static void init(CANInterface *can1_, CANInterface *can2_, AbstractAHRS *ahrs_);

    static void start_inspection(tprio_t thread_prio);
    static void start_referee_inspection(tprio_t thread_prio);

    static void startup_check_can();
    static void startup_check_mpu();
    static void startup_check_ist();
    static void startup_check_remote();
    static void startup_check_gimbal_feedback();

    static bool gimbal_failure();
    static bool remote_failure();

private:

    static AbstractAHRS *ahrs;
    static CANInterface *can1;
    static CANInterface *can2;

    static bool gimbal_failure_;
    static bool remote_failure_;

    static bool check_gimbal_failure();
    static bool check_remote_data_error();

    /// Inspector Thread
    class InspectorThread : public chibios_rt::BaseStaticThread<2048> {
        static constexpr unsigned INSPECTOR_THREAD_INTERVAL = 20;  // [ms]
        void main() final ;
    };

    static InspectorThread inspectorThread;

    /// Referee Inspector Thread
    class RefereeInspectorThread : public chibios_rt::BaseStaticThread<256> {

        event_listener_t data_received_listener;
        static constexpr eventmask_t DATA_RECEIVED_EVENTMASK = (1U << 0U);

        void main() final;
    };

    static RefereeInspectorThread refereeInspectorThread;
};


#endif //META_INFANTRY_INSPECTOR_AERIAL_H
