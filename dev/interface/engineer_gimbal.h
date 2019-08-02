//
// Created by Kerui Zhu on 7/22/2019.
//

#ifndef META_INFANTRY_ENGINEER_GIMBAL_H
#define META_INFANTRY_ENGINEER_GIMBAL_H

#include "ch.hpp"
#include "hal.h"
#include "shell.h"

class EngineerGimbalIF {

public:

    enum gimbal_id_t{
        PIT,
        YAW
    };

    static constexpr float MAX_ANGLE = 300.0f;

    static void init();

    static void set_target_angle(float yaw_angle_, float pitch_angle_);

    static float get_target_angle(gimbal_id_t id);

private:

    static float target_angle[2];
};


#endif //META_INFANTRY_ENGINEER_GIMBAL_H
