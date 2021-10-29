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
#include "vision_scheduler.h"
#include "trajectory_calculator.hpp"

GimbalLG::action_t GimbalLG::action = FORCED_RELAX_MODE;
GimbalLG::VisionControlThread GimbalLG::vision_control_thread;
GimbalLG::SentryControlThread GimbalLG::sentry_control_thread;
float GimbalLG::sub_pitch_to_ground = 0;
float GimbalLG::PITCH_MIN_ANGLE = 0;
float GimbalLG::PITCH_MAX_ANGLE = 0;
float GimbalLG::SUB_PITCH_MIN_ANGLE = 0;
float GimbalLG::SUB_PITCH_MAX_ANGLE = 0;

void GimbalLG::init(tprio_t vision_control_thread_prio, tprio_t sentry_control_thread_prio, float pitch_min_angle_,
                    float pitch_max_angle_, float sub_pitch_min_angle_, float sub_pitch_max_angle_) {
    if (vision_control_thread_prio != IDLEPRIO) {
        vision_control_thread.start(vision_control_thread_prio);
    }
    if (sentry_control_thread_prio != IDLEPRIO) {
        sentry_control_thread.start(sentry_control_thread_prio);
    }
    PITCH_MIN_ANGLE = pitch_min_angle_;
    PITCH_MAX_ANGLE = pitch_max_angle_;
    SUB_PITCH_MIN_ANGLE = sub_pitch_min_angle_;
    SUB_PITCH_MAX_ANGLE = sub_pitch_max_angle_;
}

GimbalLG::action_t GimbalLG::get_action() {
    return action;
}

void GimbalLG::set_action(GimbalLG::action_t value) {
    if (value == action) return;  // avoid repeating setting target_theta, etc.

    action = value;
    if (action == FORCED_RELAX_MODE) {
        GimbalSKD::set_mode(GimbalSKD::FORCED_RELAX_MODE);
    } else {
        GimbalSKD::set_mode(GimbalSKD::ENABLED_MODE);
    }
}

void GimbalLG::set_target(float yaw_target_angle, float pitch_target_angle, float sub_pitch_target_angle) {
    if (action != FORCED_RELAX_MODE && action != VISION_MODE) {
        GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle, sub_pitch_target_angle);
    } else {
        LOG_ERR("GimbalLG - set_target(): invalid mode");
    }
}

float GimbalLG::get_actual_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_accumulated_angle(motor);
}

float GimbalLG::get_relative_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_relative_angle(motor);
}

float GimbalLG::get_current_target_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_target_angle(motor);
}

void GimbalLG::separate_pitch() {
    sub_pitch_to_ground = GimbalSKD::get_accumulated_angle(PITCH) - GimbalSKD::get_accumulated_angle(SUB_PITCH);
}

void GimbalLG::cal_separate_angle(float &target_pitch, float &target_sub_pitch) {
    float temp_target_pitch = sub_pitch_to_ground;
    float flight_time;
    bool ret = Trajectory::compensate_for_gravity(temp_target_pitch, *GimbalIF::lidar_dist, 10, flight_time);
    if (ret) {
        target_pitch = temp_target_pitch;
        target_sub_pitch = GimbalSKD::get_accumulated_angle(PITCH) - sub_pitch_to_ground;
    }
}

void GimbalLG::cal_merge_pitch(float &target_pitch, float &target_sub_pitch) {
    target_pitch = sub_pitch_to_ground;
    target_sub_pitch = GimbalSKD::get_accumulated_angle(PITCH) - sub_pitch_to_ground;
}

void GimbalLG::VisionControlThread::main() {
    setName("GimbalLG_Vision");

    chEvtRegisterMask(&Vision::gimbal_updated_event, &vision_listener, EVENT_MASK(0));

    while (!shouldTerminate()) {

        chEvtWaitAny(ALL_EVENTS);

        if (action == VISION_MODE) {

            float yaw, pitch;
            bool can_reach_the_target;

            chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
            {
                can_reach_the_target = Vision::get_gimbal_target_angles(yaw, pitch);
            }
            chSysUnlock();  /// --- EXIT S-Locked state ---

            if (can_reach_the_target) {

                VAL_CROP(pitch, PITCH_MAX_ANGLE, PITCH_MIN_ANGLE);
                GimbalSKD::set_target_angle(yaw, pitch);

            }  // otherwise, keep current target angles
        }
    }
}

void GimbalLG::SentryControlThread::main() {
    setName("GimbalLG_Sentry");

    while (!shouldTerminate()) {

        if (action == SENTRY_MODE) {
            float yaw = GimbalSKD::get_target_angle(YAW) + 0.05f;
            time_ticket += 0.05f;
            float pitch = sin(time_ticket / 180 * PI);
            GimbalSKD::set_target_angle(yaw, pitch, 0);
        }
        sleep(TIME_MS2I(SENTRY_THREAD_INTERVAL));
    }
}

/** @} */