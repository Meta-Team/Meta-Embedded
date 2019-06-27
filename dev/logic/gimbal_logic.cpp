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

GimbalLG::action_t GimbalLG::get_action() {
    if (GimbalSKD::get_mode() == GimbalSKD::FORCED_RELAX_MODE) {
        return FORCED_RELAX_MODE;
    } else if (GimbalSKD::get_mode() == GimbalSKD::ABS_ANGLE_MODE) {
        return ABS_ANGLE_MODE;
    }
    return FORCED_RELAX_MODE;  // to avoid warning
}

void GimbalLG::set_action(GimbalLG::action_t value) {
    if (value == FORCED_RELAX_MODE) {
        GimbalSKD::set_mode(GimbalSKD::FORCED_RELAX_MODE);
    } else if (value == ABS_ANGLE_MODE) {
        GimbalSKD::set_mode(GimbalSKD::ABS_ANGLE_MODE);
    }
}

void GimbalLG::set_target(float yaw_target_angle, float pitch_target_angle) {
    GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle);
}

float GimbalLG::get_accumulated_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_accumulated_angle(motor);
}

/** @} */