//
// Created by zhukerui on 2019/6/9.
//

#ifndef META_INFANTRY_SUSPENSION_GIMBAL_CONTROLLER_H
#define META_INFANTRY_SUSPENSION_GIMBAL_CONTROLLER_H
#include "pid_controller.h"
#include "suspension_gimbal_interface.h"

class SuspensionGimbalController {
public:

    typedef enum {
        OFF = 0,
        AWAIT = 1,
        SHOOT = 2,
    } shoot_mode_t;

    static float shoot_duty_cycles[3];  // the array contains the duty cycles for different shoot modes

    /**
     * PIDController for each motor
     */

    PIDController yaw_angle_to_v;
    PIDController yaw_v_to_i;
    PIDController pitch_angle_to_v;
    PIDController pitch_v_to_i;
    PIDController BL_v_to_i;

    void start_continuous_shooting();

    void start_incontinuous_shooting(int bullet_num);

    void stop_shooting();

    float get_target_current(float measured_velocity, float target_velocity);

    void update_accumulation_angle(float accumulate_angle);

    void update_bullet(int bullet_changed);

private:

    float last_accumulate_angle = 0;

    bool continuous_shooting = false;
    bool shooting = false;
    float shoot_target_angle = 0; // in incontinuous mode, bullet loader stop if shoot_target_angle has been achieved
    float shoot_accumulate_angle = 0; // angle that is achieved during a single shoot

    /** Configurations **/
    float const one_bullet_step = 40.0; // degree

    static int remained_bullet;
};


#endif //META_INFANTRY_SUSPENSION_GIMBAL_CONTROLLER_H
