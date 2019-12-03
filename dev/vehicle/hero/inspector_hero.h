//
// Created by liuzikai on 2019-06-25.
// Edited by Qian Chen & Mo Kanya on 2019-07-05
//

#ifndef META_HERO_INSPECTOR_H
#define META_HERO_INSPECTOR_H

#include "ch.hpp"

#include "buzzer_scheduler.h"

#include "can_interface.h"
#include "ahrs_abstract.h"
#include "remote_interpreter.h"
#include "chassis_interface.h"
#include "gimbal_interface.h"
#include "referee_interface.h"

#if defined(HERO)
#include "vehicle_hero.h"
#else
#error "Files inspector_hero.h/cpp should only be used for Hero main program"
#endif

class InspectorH {

public:

    static void init(CANInterface *can1_, CANInterface *can2_, AbstractAHRS *ahrs_);

    static void start_inspection(tprio_t thread_prio, tprio_t referee_inspector_prio);

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
    class InspectorThread : public chibios_rt::BaseStaticThread<1024> {
        static constexpr unsigned INSPECTOR_THREAD_INTERVAL = 20;  // [ms]
        void main();
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


#endif //META_HERO_INSPECTOR_H
