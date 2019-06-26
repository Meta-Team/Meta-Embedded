//
// Created by liuzikai on 2019-06-26.
//

#include "gimbal_logic.h"

GimbalLG::action_t GimbalLG::get_action() {
    if (GimbalSKD::get_mode() == GimbalSKD::STOP_MODE) {
        return STOP_MODE;
    } else if (GimbalSKD::get_mode() == GimbalSKD::ABS_ANGLE_MODE) {
        return ABS_ANGLE_MODE;
    }
}

void GimbalLG::set_action(GimbalLG::action_t value) {
    if (value == STOP_MODE) {
        GimbalSKD::set_mode(GimbalSKD::STOP_MODE);
    } else if (value == ABS_ANGLE_MODE) {
        GimbalSKD::set_mode(GimbalSKD::ABS_ANGLE_MODE);
    }
}

void GimbalLG::set_target(float yaw_target_angle, float pitch_target_angle) {
    GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle);
}