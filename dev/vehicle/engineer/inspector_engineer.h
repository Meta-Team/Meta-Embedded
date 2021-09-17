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
#include "engineer_grab_mech_interface.h"

#if defined(ENGINEER)
#include "vehicle_engineer.h"
#else
#error "Files inspector_engineer.h/cpp should only be used for Engineer main program"
#endif

class InspectorE {

public:

    /**
     * @brief initiate Inspector, assign can interfaces for inspector
     * @param can1_ can address for CAN1
     * @param can2_ can address for CAN2
     */
    static void init(CANInterface *can1_, CANInterface *can2_);

    /**
     * @brief initiate inspection thread, assign thread priority
     * @param referee_inspector_prio priority for referee inspector
     * @param thread_prio priority for inspector thread priority
     */
    static void start_inspection(tprio_t thread_prio, tprio_t referee_inspector_prio);

    /**
     * @brief check can on startup, ensure can working properly.
     */
    static void startup_check_can();

    /**
     * @brief check remote on startup, ensure remote working properly.
     */
    static void startup_check_remote();

    /**
     * @brief check if board can receive can signals from chassis.
     */
    static void startup_check_chassis_feedback();

    /**
     * @brief check if board can receive can signals from robotic arm.
     */
    static void startup_check_robotic_arm_feedback();

    /**
     * @brief check if chassis has motor offline.
     * @return whether chassis has motor offline.
     */
    static bool chassis_failure();

    /**
      * @brief check if robotic arm has motor offline.
      * @return whether robotic arm has motor offline.
      */
    static bool robotic_arm_failure();

    /**
     * @brief check if remote controller is offline.
     * @return whether remtoe controller is offline.
     */
    static bool remote_failure();

private:

    static CANInterface *can1;
    static CANInterface *can2;

    static bool chassis_failure_;
    static bool robotic_arm_failure_;
    static bool remote_failure_;

    static bool check_chassis_failure();
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
