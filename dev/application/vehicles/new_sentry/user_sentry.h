//
// Created by Kerui Zhu on 2019-07-16.
//

#ifndef META_SENTRY_USER_H
#define META_SENTRY_USER_H

#include "ch.hpp"

#include "ch.hpp"

#include "remote_interpreter.h"
#include "referee_interface.h"
//#include "capacitor_interface.h"

//#include "gimbal_logic.h"
//#include "shoot_logic.h"
#include "omni_chassis_logic.h"
#include "vision_scheduler.h"

#include "inspector_sentry.h"

class UserS {
public:

    static void start(tprio_t user_thd_prio);

private:

    /// Gimbal config
    static float gimbal_rc_yaw_max_speed;  // [degree/s]
    static float gimbal_yaw_min_angle; // left range for yaw [degree]
    static float gimbal_yaw_max_angle; // right range for yaw [degree]
    static float gimbal_pitch_min_angle; // down range for pitch [degree]
    static float gimbal_pitch_max_angle; // up range for pitch [degree]

    /// Chassis config
    static float base_power;
    static float base_velocity;

    /// Chassis config
    static float chassis_v;  // [mm/s]

    /// Shoot config
    static float shoot_feed_rate;
    static float shoot_fw_speed;

    /// Runtime variables
    static float gimbal_yaw_target_angle_;
    static float gimbal_pitch_target_angle_;
    static bool ignore_shoot_constraints;

    /// User Thread
    static constexpr unsigned USER_THREAD_INTERVAL = 7;  // [ms]
    class UserThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static UserThread userThread;

//    /// Virtual User Thread
//    static constexpr unsigned VIRTUAL_USER_THREAD_INTERVAL = 7;  // [ms]
//    class VirtualUserThread : public chibios_rt::BaseStaticThread<512> {
//        void main() final;
//    };
//
//    static VirtualUserThread virtualUserThread;


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


#endif //META_SENTRY_USER_H
