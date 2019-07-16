//
// Created by liuzikai on 2019-06-25.
//

#ifndef META_SENTRY_USER_H
#define META_SENTRY_USER_H

#include "ch.hpp"

#include "remote_interpreter.h"
#include "referee_interface.h"
#include "super_capacitor_port.h"

#include "gimbal_logic.h"
#include "infantry_shoot_logic.h"
#include "sentry_chassis_logic.h"

#include "sentry_inspector.h"

class User {

public:

    static void start(tprio_t user_thd_prio);

private:

    enum gimbal_sensitivity_t{
        CRUISING,
        TARGET_SLOW,
        TARGET_FAST
    };

    /// Gimbal Config
    static float yaw_sensitivity[];  // [Cruising, Target_slow, Target_fast] [degree/s]
    static float pitch_sensitivity[];  // [Cruising, Target_slow, Target_fast] [degree/s]

    static float gimbal_yaw_min_angle; // left range for yaw [degree]
    static float gimbal_yaw_max_angle; // right range for yaw [degree]
    static float gimbal_pitch_min_angle; // down range for pitch [degree]
    static float gimbal_pitch_max_angle; //  up range for pitch [degree]

    /// Chassis Config
    static float chassis_v;  // [mm/s]

    /// Shoot Config
    static float shoot_launch_left_count;
    static float shoot_launch_right_count;

    static float shoot_launch_speed;

    static float shoot_common_duty_cycle;

    static Remote::key_t shoot_fw_switch;

    /// User Thread
    static constexpr unsigned USER_THREAD_INTERVAL = 7;  // [ms]
    class UserThread : public chibios_rt::BaseStaticThread<512> {

        /// Runtime variables

        float gimbal_yaw_target_angle_ = 0;
        float gimbal_pitch_target_angle_ = 0;

        void main() final;
    };

    static UserThread userThread;

    /// Friend Configure Functions
    friend void gimbal_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void chassis_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void chassis_set_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]);

};


#endif //META_SENTRY_USER_H
