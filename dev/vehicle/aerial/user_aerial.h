//
// Created by Kerui Zhu on 7/19/2019.
//

#ifndef META_INFANTRY_USER_AERIAL_H
#define META_INFANTRY_USER_AERIAL_H

#include "ch.hpp"

#include "remote_interpreter.h"
#include "referee_interface.h"

#include "gimbal_logic.h"
#include "shoot_logic.h"

#include "inspector_aerial.h"

class UserA {

public:

    static void start(tprio_t user_thd_prio, tprio_t user_action_thd_prio, tprio_t client_data_sending_thd_prio);

private:

    /// Gimbal Config
    static float gimbal_rc_yaw_max_speed;  // [degree/s]
    static float gimbal_pc_yaw_sensitivity[];  // [Ctrl, Normal, Shift] [degree/s]
    static float gimbal_yaw_max_angle;  // [degree]
    static float gimbal_yaw_min_angle;  // [degree]

    static float gimbal_rc_pitch_max_speed;  // [degree/s]
    static float gimbal_pc_pitch_sensitivity[];   // rotation speed when mouse moves fastest [degree/s]
    static float gimbal_pitch_min_angle; // down range for pitch [degree]
    static float gimbal_pitch_max_angle; //  up range for pitch [degree]

    /// Shoot Config
    static bool shoot_power_on;
    static float shoot_launch_left_count;
    static float shoot_launch_right_count;

    static float shoot_launch_speed;

    static float shoot_common_duty_cycle;
    static float shoot_debug_duty_cycle;
    static float shoot_full_power_duty_cycle;

    static Remote::key_t shoot_fw_switch;

    /// Helpers
    static void set_user_client_speed_light_(int level);

    /// Runtime variables

    static float gimbal_yaw_target_angle_;
    static float gimbal_pitch_target_angle_;

    /// User Thread
    static constexpr unsigned USER_THREAD_INTERVAL = 7;  // [ms]
    class UserThread : public chibios_rt::BaseStaticThread<512> {
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
    friend void gimbal_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]);
};


#endif //META_INFANTRY_USER_AERIAL_H
