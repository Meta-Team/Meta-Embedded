//
// Created by Kerui Zhu on 7/22/2019.
//

#ifndef META_INFANTRY_ENGINEER_GIMBAL_H
#define META_INFANTRY_ENGINEER_GIMBAL_H

#include "ch.hpp"
#include "hal.h"

class EngineerGimbalIF {

public:

    static constexpr float MAX_ANGLE = 300.0f;

    static void init();

    static void set_target_angle(float yaw_angle_, float pitch_angle_);

    enum gimbal_t{
        YAW,
        PIT
    };

    static float yaw_angle;

    static float pitch_angle;

    static void send_current();

};


#endif //META_INFANTRY_ENGINEER_GIMBAL_H
