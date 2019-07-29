//
// Created by liuzikai on 2019-06-26.
//

/**
 * @file    gimbal_logic.cpp
 * @brief   By-passing logic-level module to control GimbalLG
 *
 * @addtogroup gimbal
 * @{
 */

#include "gimbal_logic.h"

GimbalLG::action_t GimbalLG::action = FORCED_RELAX_MODE;
float GimbalLG::pitch_max_angle = 0;
float GimbalLG::pitch_min_angle = 0;
float GimbalLG::yaw_max_angle = 0;
float GimbalLG::yaw_min_angle = 0;

void GimbalLG::init(float yaw_min_angle_, float yaw_max_angle_,
                    float pitch_min_angle_, float pitch_max_angle_) {
    yaw_min_angle = yaw_min_angle_;
    yaw_max_angle = yaw_max_angle_;
    pitch_min_angle = pitch_min_angle_;
    pitch_max_angle = pitch_max_angle_;
}

GimbalLG::action_t GimbalLG::get_action() {
    return action;
}

void GimbalLG::set_action(GimbalLG::action_t value) {
    if (value == action) return;  // avoid repeating setting target_theta, etc.

    action = value;
    if (action == FORCED_RELAX_MODE) {
        GimbalSKD::set_mode(GimbalSKD::FORCED_RELAX_MODE);
    } else if (action == ABS_ANGLE_MODE || action == AERIAL_MODE) {
        GimbalSKD::set_mode(GimbalSKD::ABS_ANGLE_MODE);
    } else if (action == SENTRY_MODE) {
        GimbalSKD::set_mode(GimbalSKD::SENTRY_MODE);
    }
}

void GimbalLG::set_target(float yaw_target_angle, float pitch_target_angle) {
    if (action == ABS_ANGLE_MODE || action == SENTRY_MODE) {

        GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle);

    } else if (action == AERIAL_MODE) {
        // Compare the old ones with new ones

        float orig_yaw_target_angle = GimbalSKD::get_target_angle(YAW);
        float orig_pitch_target_angle = GimbalSKD::get_target_angle(PITCH);

        if ((yaw_target_angle < orig_yaw_target_angle &&  // to decrease, and
             GimbalSKD::get_accumulated_angle(YAW) < yaw_min_angle + AERIAL_LIMIT_ANGLE_TOLORANCE  // no enough space
            ) ||  // or
            (yaw_target_angle > orig_yaw_target_angle &&  // to increase, and
             GimbalSKD::get_accumulated_angle(YAW) > yaw_max_angle - AERIAL_LIMIT_ANGLE_TOLORANCE  // no enough space
            )) {
            yaw_target_angle = orig_yaw_target_angle;  // give up change
        }

        if ((pitch_target_angle < orig_pitch_target_angle &&  // to decrease, and
             GimbalSKD::get_accumulated_angle(PITCH) < pitch_min_angle + AERIAL_LIMIT_ANGLE_TOLORANCE  // no enough space
            ) ||  // or
            (pitch_target_angle > orig_pitch_target_angle &&  // to increase, and
             GimbalSKD::get_accumulated_angle(PITCH) > pitch_max_angle - AERIAL_LIMIT_ANGLE_TOLORANCE  // no enough space
            )) {
            pitch_target_angle = orig_pitch_target_angle;  // give up change
        }

        GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle);

    } else {
        LOG_ERR("GimbalLG - set_target(): invalid mode");
    }
}

float GimbalLG::get_accumulated_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_accumulated_angle(motor);
}

/** @} */