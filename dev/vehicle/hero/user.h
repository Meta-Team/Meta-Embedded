//
// Created by liuzikai on 2019-06-25.
// Edited by Qian Chen & Mo Kanya on 2019-07-05
//

#ifndef META_HERO_USER_H
#define META_HERO_USER_H

#include "ch.hpp"

#include "remote_interpreter.h"

#include "gimbal_logic.h"
#include "hero_shoot_logic.h"
#include "chassis_logic.h"

#include "inspector.h"

class User {

public:

    static void start(tprio_t prio);

private:

    /// Gimbal Parameters
    static constexpr float GIMBAL_RC_YAW_MAX_SPEED = 90;  // [degree/s]
    static constexpr float GIMBAL_PC_YAW_SENSITIVITY = 54000;     // rotation speed when mouse moves fastest [degree/s]

    static constexpr float GIMBAL_PC_PITCH_SENSITIVITY = 12000;   // rotation speed when mouse moves fastest [degree/s]
    static constexpr float GIMBAL_PITCH_MIN_ANGLE = -10; // down range for pitch [degree]
    static constexpr float GIMBAL_PITCH_MAX_ANGLE = 45; //  up range for pitch [degree]

    /// Shoot Parameters
    static constexpr int SHOOT_LAUNCH_LEFT_COUNT = 5;
    static constexpr int SHOOT_LAUNCH_RIGHT_COUNT = 999;

    static constexpr float SHOOT_COMMON_DUTY_CYCLE = 0.8;

    /// Chassis Parameters
    static constexpr float CHASSIS_COMMON_VX = 1200.0f;  // [mm/s]
    static constexpr float CHASSIS_COMMON_VY = 1200.0f;  // [mm/s]

    static constexpr float CHASSIS_PC_CTRL_RATIO = 0.5f;  // 50% when Ctrl is pressed


    /// Thread
    static constexpr unsigned USER_THREAD_INTERVAL = 7;  // [ms]

    class UserThread : public chibios_rt::BaseStaticThread<512> {
        float gimbal_rc_yaw_target_angle = 0;
        float gimbal_pc_yaw_target_angle = 0;
        float gimbal_pc_pitch_target_angle = 0;
        bool pc_x_pressed = false;
        bool pc_z_pressed = false;
        bool pc_mouse_left_pressed = false;
        bool pc_mouse_right_pressed = false;
        void main() final;
    };

    static UserThread userThread;

};


#endif //META_INFANTRY_HERO_H