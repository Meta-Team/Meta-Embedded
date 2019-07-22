//
// Created by liuzikai on 2019-06-25.
//

#ifndef META_INFANTRY_INSPECTOR_INFANTRY_H
#define META_INFANTRY_INSPECTOR_INFANTRY_H

#include "ch.hpp"

#include "buzzer.h"

#include "can_interface.h"
#include "remote_interpreter.h"
#include "chassis_interface.h"
#include "engineer_elevator_interface.h"

#if defined(ENGINEER)
#include "vehicle_engineer.h"
#else
#error "Files inspector_engineer.h/cpp should only be used for Engineer main program"
#endif

class InspectorE {

public:

    static void init(CANInterface *can1_, CANInterface *can2_);

    static void start_inspection(tprio_t thread_prio);

    static void startup_check_can();
    static void startup_check_remote();
    static void startup_check_chassis_feedback();
    static void startup_check_elevator_feedback();

    static bool chassis_failure();
    static bool elevator_failure();
    static bool remote_failure();

private:

    static CANInterface *can1;
    static CANInterface *can2;

    static bool chassis_failure_;
    static bool elevator_failure_;
    static bool remote_failure_;

    static bool check_chassis_failure();
    static bool check_elevator_failure();
    static bool check_remote_data_error();

    /// Inspector Thread
    class InspectorThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned INSPECTOR_THREAD_INTERVAL = 20;  // [ms]
        void main();
    };

    static InspectorThread inspectorThread;

};


#endif //META_INFANTRY_INSPECTOR_INFANTRY_H
