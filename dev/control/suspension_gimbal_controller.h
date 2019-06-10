//
// Created by zhukerui on 2019/6/9.
//

#ifndef META_INFANTRY_SUSPENSION_GIMBAL_CONTROLLER_H
#define META_INFANTRY_SUSPENSION_GIMBAL_CONTROLLER_H

#include "pid_controller.h"
#include "suspension_gimbal_interface.h"

class SuspensionGimbalController {
public:
    /**
     * PIDController for each motor
     */
    static PIDController yaw_angle_to_v;
    static PIDController yaw_v_to_i;
    static PIDController pitch_angle_to_v;
    static PIDController pitch_v_to_i;
    static PIDController BL_v_to_i;
    static float bullet_loader_speed; // degrees/s
    static float target_yaw_angle;
    static float target_pitch_angle;

    static void set_shoot_mode(SuspensionGimbalIF::shoot_mode_t mode);

    static void set_motor_enable(SuspensionGimbalIF::motor_id_t motor_id, bool status);

    static void start_continuous_shooting();

    static void stop_continuous_shooting();

    static void start_incontinuous_shooting(int bullet_num);

    static void set_target_signal();

    static void set_target_signal(SuspensionGimbalIF::motor_id_t motor, int signal);

private:

    static bool continuous_shooting;
    static float shoot_target_angle; // in incontinuous mode, bullet loader stop if shoot_target_angle has been achieved

    /** Configurations **/
    static constexpr float one_bullet_step = 40.0f; // degree
};


#endif //META_INFANTRY_SUSPENSION_GIMBAL_CONTROLLER_H
