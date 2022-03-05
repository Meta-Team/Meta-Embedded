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

GimbalLG::mode_t GimbalLG::mode;

void GimbalLG::init(tprio_t vision_prio, tprio_t gimbal_control_prio) {

}

void GimbalLG::set_target_angle(float yaw_target_angle, float pitch_target_angle) {
    GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle, -pitch_target_angle);
}

void GimbalLG::set_action(GimbalLG::mode_t mode_) {
    mode = mode_;
    /// TODO: Add gimbal ref mode
    if(mode_ == FORCED_RELAX_MODE) {
        GimbalSKD::set_mode(GimbalSKD::FORCED_RELAX_MODE);
    } else {
        GimbalSKD::set_mode(GimbalSKD::CHASSIS_REF_MODE);
    }
}

#if defined(ENABLE_SUBPITCH)
void GimbalLG::GimbalSubPitchThd::main() {
    setName("SubPitchThd");
    while(!shouldTerminate()) {
        sleep(INTERVAL);
    }
}
#endif
/** @} */