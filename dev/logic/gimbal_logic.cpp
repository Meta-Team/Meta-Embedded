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
/** TODO: to be moved out of here **/
#define PITCH_MAX_ANGLE 30
#define PITCH_MIN_ANGLE -30
/** Todo: to be removed here**/
GimbalLG::mode_t GimbalLG::mode;
#if ENABLE_VISION == TRUE
GimbalLG::VisionControlThread GimbalLG::vision_control_thread;
#endif

#if ENABLE_SUBPITCH == TRUE
GimbalLG::BallisticCompensateThread GimbalLG::ballistic_compensate_thread;
#endif

void GimbalLG::init(tprio_t vision_prio, tprio_t ballistic_compensate_prio) {
#if ENABLE_VISION == TRUE
    vision_control_thread.start(vision_prio);
#endif
#if ENABLE_SUBPITCH == TRUE
    ballistic_compensate_thread.start(ballistic_compensate_prio);
#endif
}

void GimbalLG::set_target_angle(float yaw_target_angle, float pitch_target_angle) {
    GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle, -pitch_target_angle);
}

void GimbalLG::set_mode(GimbalLG::mode_t mode_) {
    mode = mode_;
    if(mode_ == GimbalLG::FORCED_RELAX_MODE) {
        GimbalSKD::set_mode(GimbalSKD::FORCED_RELAX_MODE);
    } else if(mode_ == GimbalLG::CHASSIS_REF_MODE){
        GimbalSKD::set_mode(GimbalSKD::CHASSIS_REF_MODE);
    }
#if ENABLE_AHRS
    else if(mode_ == GimbalLG::GIMBAL_REF_MODE) {
        GimbalSKD::set_mode(GimbalSKD::GIMBAL_REF_MODE);
    }
#endif
}

float GimbalLG::get_feedback_angle(GimbalSKD::angle_id_t angle) {
    return GimbalSKD::get_feedback_angle(angle);
}

float GimbalLG::get_motor_angle(GimbalSKD::angle_id_t angle) {
    switch (angle) {
        case GimbalSKD::YAW:
            return CANMotorIF::motor_feedback[CANMotorCFG::YAW].accumulate_angle();
        case GimbalSKD::PITCH:
            return CANMotorIF::motor_feedback[CANMotorCFG::PITCH].accumulate_angle();
#if ENABLE_SUBPITCH == TRUE
        case GimbalSKD::SUB_PITCH:
            return CANMotorIF::motor_feedback[CANMotorCFG::SUB_PITCH].accumulate_angle();
#endif
        case GimbalSKD::GIMBAL_MOTOR_COUNT:
            break;
    }
    return 0.0f;
}

float GimbalLG::get_target_angle(GimbalSKD::angle_id_t angle) {
    return GimbalSKD::get_target_angle(angle);
}

float GimbalLG::get_feedback_velocity(GimbalSKD::angle_id_t angle) {
    return GimbalSKD::get_feedback_velocity(angle);
}

#if ENABLE_VISION == TRUE
void GimbalLG::VisionControlThread::main() {
    setName("GimbalLG_Vision");

    chEvtRegisterMask(&VisionSKD::gimbal_updated_event, &vision_listener, EVENT_MASK(0));

    while (!shouldTerminate()) {

        chEvtWaitAny(ALL_EVENTS);

        if (mode == VISION_MODE) {

            float yaw, pitch;
            bool can_reach_the_target;

            chSysLock();  /// --- ENTER S-Locked state ---
            {
                can_reach_the_target = VisionSKD::get_gimbal_target_angles(yaw, pitch);
            }
            chSysUnlock();  /// --- EXIT S-Locked state ---

            if (can_reach_the_target) {
                /// TODO: define the angles in vehicles.h
                VAL_CROP(pitch, PITCH_MAX_ANGLE, PITCH_MIN_ANGLE);
                GimbalSKD::set_target_angle(yaw, pitch);

            }  // otherwise, keep current target angles
        }
    }
}
#endif

#if ENABLE_SUBPITCH == TRUE
void GimbalLG::BallisticCompensateThread::main() {
    setName("SubPitchThd");
    while(!shouldTerminate()) {
        sleep(INTERVAL);
    }
}
#endif
/** @} */