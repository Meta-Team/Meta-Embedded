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
chibios_rt::ThreadReference GimbalLG::vision_control_thread_reference;


void GimbalLG::init(tprio_t vision_control_thread_prio) {
    vision_control_thread.started = true;
    vision_control_thread_reference = vision_control_thread.start(vision_control_thread_prio);
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
//        GimbalSKD::set_mode(GimbalSKD::SENTRY_MODE);
    } else if (action == VISION_MODE) {
        GimbalSKD::set_mode(GimbalSKD::ABS_ANGLE_MODE);
        // Resume the thread
        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            if (!vision_control_thread.started) {
                vision_control_thread.started = true;
                chSchWakeupS(vision_control_thread_reference.getInner(), 0);
            }
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---
    }
}

void GimbalLG::set_target(float yaw_target_angle, float pitch_target_angle) {
    if (action != FORCED_RELAX_MODE && action != VISION_MODE) {
        GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle, 0);
    } else {
        LOG_ERR("GimbalLG - set_target(): invalid mode");
    }
}

//void GimbalLG::set_target_with_sub_pitch(float yaw_target_angle, float pitch_target_angle, float sub_pitch_target_angle) {
//    if (action != FORCED_RELAX_MODE && action != VISION_MODE) {
//        GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle);
//        SubPitchSKD::set_target_angle(sub_pitch_target_angle);
//    } else {
//        LOG_ERR("GimbalLG - set_target(): invalid mode");
//    }
//}

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
    setName("GimbalIF_Vision");

    while(!shouldTerminate()) {
        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            if (action != VISION_MODE) {
                started = false;
                chSchGoSleepS(CH_STATE_SUSPENDED);
            }
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        Vision::VisionControlCommand command = {0, 0};
        if (Vision::getControlCommand(command)) {
            GimbalSKD::set_target_angle(command.gimbal_yaw_target, command.gimbal_pitch_target, 0);
        }  // otherwise, keep current target angles

        sleep(TIME_MS2I(VISION_THREAD_INTERVAL));
    }
}

/** @} */