//
// Created by liuzikai on 2019-06-25.
//

#ifndef META_SENTRY_USER_INFANTRY_H
#define META_SENTRY_USER_INFANTRY_H

#include "ch.hpp"

#include "remote_interpreter.h"
//#include "referee_interface.h"
#include "capacitor_interface.h"

//#include "gimbal_logic.h"
//#include "shoot_logic.h"
#include "omni_chassis_logic.h"
#include "vision_scheduler.h"

#include "inspector_sentry.h"

class UserS {

public:

    static void start(tprio_t user_thd_prio, tprio_t user_action_thd_prio);

private:

    /// Gimbal config
    static float gimbal_rc_yaw_max_speed;  // [degree/s]
    static float gimbal_pc_yaw_sensitivity[];  // [Ctrl, Normal, Shift] [degree/s]

    static float gimbal_pc_pitch_sensitivity[];   // rotation speed when mouse moves fastest [degree/s]

    /// Chassis config
    static float base_power;             // [w]
    static float base_v_forward;        // [mm/s]
    static float chassis_v_left_right;  // [mm/s]
    static float Base_left_right_power; // [w]
    static  float Base_left_right;      // [mm/s]
    static float chassis_v_forward;     // [mm/s]
    static float chassis_v_backward;    // [mm/s]

    static float chassis_pc_shift_ratio;  // 150% when Shift is pressed
    static float chassis_pc_ctrl_ratio;    // 50% when Ctrl is pressed

    /// Shoot config
    static float shoot_feed_rate;
    static float shoot_fw_speed[3];

    static bool mag_status;

    /// Helpers

    /// Runtime variables

    static float gimbal_yaw_target_angle_;
    static float gimbal_pc_pitch_target_angle_;
    static bool ignore_shoot_constraints;

    /// User thread
    static constexpr unsigned USER_THREAD_INTERVAL = 7;  // [ms]
    class UserThread : public chibios_rt::BaseStaticThread<512> {
        void main() override;
    };

    static UserThread userThread;


    /// User action thread
    class UserActionThread : public chibios_rt::BaseStaticThread<512> {
        event_listener_t key_press_listener;
        void main() override;
    };

    static UserActionThread userActionThread;
#if FALSE
    /// Friend Configure Functions
    friend void gimbal_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void chassis_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void chassis_set_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]);
#endif
};


#endif //META_SENTRY_USER_INFANTRY_H