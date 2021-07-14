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
GimbalLG::VisionControlThread GimbalLG::vision_control_thread;

void GimbalLG::init(tprio_t vision_control_thread_prio) {
    vision_control_thread.start(vision_control_thread_prio);
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
    } else if (action == VISION_MODE) {
        GimbalSKD::set_mode(GimbalSKD::ABS_ANGLE_MODE);
        // Variable action is checked in the vision thread
    }
}

void GimbalLG::set_target(float yaw_target_angle, float pitch_target_angle) {
    if (action != FORCED_RELAX_MODE && action != VISION_MODE) {
        GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle);
    } else {
        LOG_ERR("GimbalLG - set_target(): invalid mode");
    }
}

float GimbalLG::get_accumulated_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_accumulated_angle(motor);
}

float GimbalLG::get_relative_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_relative_angle(motor);
}

float GimbalLG::get_current_target_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_target_angle(motor);
}

void GimbalLG::VisionControlThread::main() {
    setName("GimbalLG_Vision");

    chEvtRegisterMask(&Vision::gimbal_updated_event, &vision_listener, VISION_UPDATED_EVENT_MASK);

    while (!shouldTerminate()) {

        chEvtWaitAny(VISION_UPDATED_EVENT_MASK);

        if (action == VISION_MODE) {
            float yaw, pitch;
            bool can_reach_the_target;

            chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
            {
                can_reach_the_target = Vision::get_gimbal_target_angles(yaw, pitch);
            }
            chSysUnlock();  /// --- EXIT S-Locked state ---

            if (can_reach_the_target) {
                GimbalSKD::set_target_angle(yaw, pitch);
            }  // otherwise, keep current target angles
        }
    }
}

/** @} */