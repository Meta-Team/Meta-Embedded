//
// Created by liuzikai on 2019-06-26.
//

#ifndef META_INFANTRY_GIMBAL_LOGIC_H
#define META_INFANTRY_GIMBAL_LOGIC_H

#include "gimbal_scheduler.h"

class GimbalLG {

    enum action_t {
        STOP_MODE,
        ABS_ANGLE_MODE
    };

    static action_t get_action();

    static void set_action(action_t value);

    static void set_target(float yaw_target_angle, float pitch_target_angle);

};


#endif //META_INFANTRY_GIMBAL_LOGIC_H
