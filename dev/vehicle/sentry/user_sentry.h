//
// Created by Kerui Zhu on 2019-07-16.
//

#ifndef META_SENTRY_USER_H
#define META_SENTRY_USER_H

#include "ch.hpp"

#include "remote_interpreter.h"
#include "referee_interface.h"
#include "super_capacitor_port.h"

#include "gimbal_logic.h"
#include "shoot_logic.h"
#include "sentry_chassis_logic.h"

#include "inspector_sentry.h"
#include "vision_port.h"

class UserS {

public:

    static time_msecs_t blind_mode_start_time;

    static void start(tprio_t user_thd_prio, tprio_t v_user_thd_prio);

private:

    enum sentry_mode_t{
        FORCED_RELAX_MODE,
        REMOTE_MODE,
        AUTO_MODE
    };

    static sentry_mode_t sentryMode;

    enum gimbal_sensitivity_t{
        CRUISING,
        TARGET_SLOW,
        TARGET_FAST
    };

    /// Gimbal Config
    static float yaw_sensitivity[];  // [Cruising, Target_slow, Target_fast] [degree/s]
    static float pitch_sensitivity[];  // [Cruising, Target_slow, Target_fast] [degree/s]

    static float gimbal_yaw_target_angle_;
    static float gimbal_pitch_target_angle_;

    static float gimbal_yaw_min_angle; // left range for yaw [degree]
    static float gimbal_yaw_max_angle; // right range for yaw [degree]
    static float gimbal_pitch_min_angle; // down range for pitch [degree]
    static float gimbal_pitch_max_angle; // up range for pitch [degree]

    /// Chassis Config
    static float chassis_v;  // [mm/s]

    /// Shoot Config
    static bool shoot_power_on;
    static float shoot_launch_left_count;
    static float shoot_launch_right_count;

    static float shoot_launch_speed;

    static float shoot_common_duty_cycle;

    static void set_mode(sentry_mode_t mode);

    static bool fire;

    /// User Thread
    static constexpr unsigned USER_THREAD_INTERVAL = 7;  // [ms]
    class UserThread : public chibios_rt::BaseStaticThread<512> {

        void main() final;
    };

    static UserThread userThread;

    class VitualUserThread : public chibios_rt::BaseStaticThread<2048> {
    public:

        enum vitual_user_mode_t{
            CRUISING_ONLY_MODE,
            FINAL_AUTO_MODE
        };

        void set_v_user_mode(vitual_user_mode_t mode);

        bool started = false;

    private:
        bool enemy_spotted = false;
        vitual_user_mode_t v_user_mode = FINAL_AUTO_MODE;
        float yaw_terminal = gimbal_yaw_max_angle, pitch_terminal = gimbal_pitch_max_angle;
        static constexpr unsigned AUTO_CONTROL_INTERVAL = 5;
        static constexpr float GIMBAL_YAW_TARGET_FAST_TRIGGER = 10.0f;
        static constexpr float GIMBAL_PITCH_TARGET_FAST_TRIGGER = 5.0f;
        static constexpr float GIMBAL_YAW_SHOOT_TRIGGER_ANGLE = 30.0f;
        static constexpr float GIMBAL_PIT_SHOOT_TRIGGER_ANGLE = 10.0f;

        void main() final;
    };

    static VitualUserThread vitualUserThread;
    static chibios_rt::ThreadReference vitualUserThreadReference;


    /// Friend Configure Functions
    friend void gimbal_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void gimbal_set_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void chassis_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void chassis_set_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void shoot_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void shoot_set_config(BaseSequentialStream *chp, int argc, char *argv[]);

};


#endif //META_SENTRY_USER_H
