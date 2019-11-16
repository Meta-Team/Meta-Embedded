//
// Created by liuzikai on 2019-06-25.
//

#ifndef META_INFANTRY_INSPECTOR_INFANTRY_H
#define META_INFANTRY_INSPECTOR_INFANTRY_H

#include "ch.hpp"

#include "buzzer_scheduler.h"

#include "can_interface.h"
#include "remote_interpreter.h"
#include "referee_interface.h"

#include "chassis_interface.h"
#include "engineer_elevator_interface.h"
#include "robotic_arm_interface.h"

#if defined(ENGINEER)
#include "vehicle_engineer.h"
#else
#error "Files inspector_engineer.h/cpp should only be used for Engineer main program"
#endif

class InspectorE {

public:

    static void init(CANInterface *can1_, CANInterface *can2_);

    static void start_inspection(tprio_t thread_prio, tprio_t referee_inspector_prio);

    static void startup_check_can();
    static void startup_check_remote();
    static void startup_check_chassis_feedback();
    static void startup_check_elevator_feedback();
    static void startup_check_robotic_arm_feedback();

    static bool chassis_failure();
    static bool elevator_failure();
    static bool robotic_arm_failure();
    static bool remote_failure();

private:

    static CANInterface *can1;
    static CANInterface *can2;

    static bool chassis_failure_;
    static bool elevator_failure_;
    static bool robotic_arm_failure_;
    static bool remote_failure_;

    static bool check_chassis_failure();
    static bool check_elevator_failure();
    static bool check_robotic_arm_failure();
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


#endif //META_INFANTRY_INSPECTOR_INFANTRY_H
