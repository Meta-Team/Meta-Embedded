//
// Created by liuzikai on 2019-06-25.
//

#ifndef META_INFANTRY_USER_INFANTRY_H
#define META_INFANTRY_USER_INFANTRY_H

#include "ch.hpp"
#include "led.h"

#include "remote_interpreter.h"
#include "referee_interface.h"
#include "engineer_interface.h"

#include "engineer_elevator_logic.h"
#include "engineer_chassis_skd.h"
#include "engineer_gimbal.h"
#include "robotic_arm_skd.h"

#include "inspector_engineer.h"


class UserE {

public:

    static void start(tprio_t user_thd_prio, tprio_t user_action_thd_prio, tprio_t client_data_sending_thd_prio);

private:

    /// Chassis Config
    static float chassis_v_left_right;  // [mm/s]
    static float chassis_v_forward;     // [mm/s]
    static float chassis_v_backward;    // [mm/s]
    static float chassis_w;             // [mm/s]

    static float chassis_pc_shift_ratio;  // 150% when Shift is pressed
    static float chassis_pc_ctrl_ratio;   // 20% when Ctrl is pressed

    /// Helpers
    static void set_user_client_speed_light_(int level);

    /// Runtime variables

    static float gimbal_pc_yaw_target_angle_;
    static float gimbal_pc_pitch_target_angle_;

    /// User Thread
    static constexpr unsigned USER_THREAD_INTERVAL = 7;  // [ms]
    class UserThread : public chibios_rt::BaseStaticThread<1024> {
        void main() final;
    };

    static UserThread userThread;


    /// User Action Thread
    class UserActionThread : public chibios_rt::BaseStaticThread<512> {

        /// Runtime variables
        event_listener_t s_change_listener;
        static constexpr eventmask_t S_CHANGE_EVENTMASK = (1U << 0U);

        event_listener_t mouse_press_listener;
        static constexpr eventmask_t MOUSE_PRESS_EVENTMASK = (1U << 1U);

        event_listener_t mouse_release_listener;
        static constexpr eventmask_t MOUSE_RELEASE_EVENTMASK = (1U << 2U);

        event_listener_t key_press_listener;
        static constexpr eventmask_t KEY_PRESS_EVENTMASK = (1U << 3U);

        event_listener_t key_release_listener;
        static constexpr eventmask_t KEY_RELEASE_EVENTMASK = (1U << 4U);

        void main() final;
    };

    static UserActionThread userActionThread;

    /// Referee Client Data Sending Thread
    static constexpr unsigned CLIENT_DATA_SENDING_THREAD_INTERVAL = 100;  // [ms]

    class ClientDataSendingThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static ClientDataSendingThread clientDataSendingThread;

    /// Friend Configure Functions

};


#endif //META_INFANTRY_USER_INFANTRY_H
