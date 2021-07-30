//
// Created by liuzikai on 2019-06-26.
//

/**
 * @file    gimbal_logic.h
 * @brief   By-passing logic-level module to control GimbalLG
 *
 * @addtogroup gimbal
 * @{
 */

#ifndef META_INFANTRY_GIMBAL_LOGIC_H
#define META_INFANTRY_GIMBAL_LOGIC_H

#include "gimbal_scheduler.h"

/**
 * @name GimbalLG
 * @note LG stands for "logic"
 * @brief By-passing logic-level module to control GimbalLG
 * @pre GimbalSKD has started properly
 * @usage Invoke set_action() and set_target() to control gimbal
 */
class GimbalLG : public GimbalBase {

public:

    /**
     * Initial GimbalLG
     * @param vision_control_thread_prio  Vision control thread priority (IDLEPRIO for disabled)
     * @param sentry_control_thread_prio  Sentry control thread priority (IDLEPRIO for disabled)
     * @param pitch_min_angle
     * @param pitch_max_angle
     * @param sub_pitch_min_angle
     * @param sub_pitch_max_angle
     */
    static void init(tprio_t vision_control_thread_prio, tprio_t sentry_control_thread_prio,
                     float pitch_min_angle, float pitch_max_angle, float sub_pitch_min_angle, float sub_pitch_max_angle);

    enum action_t {
        FORCED_RELAX_MODE,
        ABS_ANGLE_MODE,
        SENTRY_MODE,
        AERIAL_MODE,
        VISION_MODE,
    };

    /**
     * Get current action of gimbal.
     * @return   Current action of gimbal
     */
    static action_t get_action();

    /**
     * Set action of gimbal.
     * @param value   Action to be applied
     */
    static void set_action(action_t value);

    /**
     * Set target angles in ABS_ANGLE_MODE.
     * @param yaw_target_angle    Yaw target ACCUMULATED angle on ground coordinate [degree]
     * @param pitch_target_angle  Pitch target ACCUMULATED angle on ground coordinate [degree]
     */
    static void set_target(float yaw_target_angle, float pitch_target_angle, float sub_pitch_target_angle = 0);

    /**
     * Get actual angle maintained by GimbalSKD.
     * @param motor   YAW or PITCH
     * @return Actual angle of the motor
     */
    static float get_actual_angle(motor_id_t motor);

    /**
    * Get relative angle maintained by GimbalSKD.
    * @param motor   YAW or PITCH
    * @return Accumulated angle of the motor
    */
    static float get_relative_angle(motor_id_t motor);

    /**
     * Get current target angle maintained by GimbalSKD.
     * @param motor   YAW or PITCH
     * @return Current target angle involved in the PID calculation.
     */
    static float get_current_target_angle(motor_id_t motor);

    static void separate_pitch();

    static void cal_separate_angle(float &target_pitch, float &target_sub_pitch);

    static void cal_merge_pitch(float &target_pitch, float &target_sub_pitch);

private:

    static action_t action;
    static float PITCH_MIN_ANGLE;
    static float PITCH_MAX_ANGLE;
    static float SUB_PITCH_MIN_ANGLE;
    static float SUB_PITCH_MAX_ANGLE;

    static float sub_pitch_to_ground;

    class VisionControlThread : public chibios_rt::BaseStaticThread<256> {
        event_listener_t vision_listener;
        void main() final;
    };

    static VisionControlThread vision_control_thread;

    class SentryControlThread : public chibios_rt::BaseStaticThread<256> {
        float time_ticket = 0;
        static constexpr unsigned int SENTRY_THREAD_INTERVAL = 5; // [ms]
        void main() final;
    };

    static SentryControlThread sentry_control_thread;

};

#endif //META_INFANTRY_GIMBAL_LOGIC_H

/** @} */