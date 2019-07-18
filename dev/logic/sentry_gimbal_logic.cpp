//
// Created by Kerui Zhu on 7/17/2019.
//

#include "sentry_gimbal_logic.h"

SGimbalLG::action_t SGimbalLG::action;

void SGimbalLG::init() {}

void SGimbalLG::set_action(action_t value){
    if (value == action) return;  // avoid repeating setting target_theta, etc.

    action = value;
    if (action == FORCED_RELAX_MODE) {
        GimbalSKD::set_mode(GimbalSKD::FORCED_RELAX_MODE);
    } else {
        GimbalSKD::set_mode(GimbalSKD::ABS_ANGLE_MODE);
    }
}

void SGimbalLG::set_target(float yaw_target_angle, float pitch_target_angle) {
    if (action == ABS_ANGLE_MODE) {
        GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle);
    } else {
        LOG_ERR("GimbalLG - set_target(): invalid mode");
    }
}

float SGimbalLG::get_accumulated_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_accumulated_angle(motor);
}