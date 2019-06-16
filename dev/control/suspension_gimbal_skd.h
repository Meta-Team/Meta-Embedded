//
// Created by zhukerui on 2019/6/9.
//

#ifndef META_INFANTRY_SUSPENSION_GIMBAL_CONTROLLER_H
#define META_INFANTRY_SUSPENSION_GIMBAL_CONTROLLER_H

#include "pid_controller.hpp"
#include "suspension_gimbal_interface.h"
#include "vehicle/sentry/vehicle_sentry.h"
#include "common_macro.h"

class SuspensionGimbalSKD{
public:
    /**
     * PIDController for each motor
     */
    static PIDController yaw_a2v_pid;
    static PIDController yaw_v2i_pid;
    static PIDController pitch_a2v_pid;
    static PIDController pitch_v2i_pid;
    static PIDController BL_v2i_pid;

    class SuspensionGimbalThread: public chibios_rt::BaseStaticThread<512>{
        void main() final;
    };

    static SuspensionGimbalThread suspensionGimbalThread;

    static void init();

    static void set_front(SuspensionGimbalIF::motor_id_t motor_id);

    static void set_shoot_mode(shoot_mode_t mode);

    static void set_motor_enable(SuspensionGimbalIF::motor_id_t motor_id, bool status);

    static void start_continuous_shooting();

    static void stop_continuous_shooting();

    static void start_incontinuous_shooting(int bullet_num);

    static void set_target_signal(SuspensionGimbalIF::motor_id_t motor, int16_t signal);

private:

    static bool continuous_shooting;

    static void set_target_signal();

    /** Configurations **/
    static constexpr float one_bullet_step = 40.0f; // degree
};


#endif //META_INFANTRY_SUSPENSION_GIMBAL_CONTROLLER_H
