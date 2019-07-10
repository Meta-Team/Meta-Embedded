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

#include "vision_port.h"

/**
 * @name GimbalLG
 * @note LG stands for "logic"
 * @brief By-passing logic-level module to control GimbalLG
 * @pre GimbalSKD has started properly
 * @usage Invoke set_action() and set_target() to control gimbal
 */
class GimbalLG : public GimbalBase {

public:

    static void init(tprio_t vision_thread_prio_);

    enum action_t {
        FORCED_RELAX_MODE,
        ABS_ANGLE_MODE,
        VISION_MODE
    };

    /**
     * Get current action of gimbal
     * @return   Current action of gimbal
     */
    static action_t get_action();

    /**
     * Set action of gimbal
     * @param value   Action to be applied
     */
    static void set_action(action_t value);

    /**
     * Set target angles in ABS_ANGLE_MODE
     * @param yaw_target_angle    Yaw target ACCUMULATED angle on ground coordinate [degree]
     * @param pitch_target_angle  Pitch target ACCUMULATED angle on ground coordinate [degree]
     */
    static void set_target(float yaw_target_angle, float pitch_target_angle);

    /**
     * Get accumulated angle maintained by this SKD
     * @param motor   YAW or PITCH
     * @return Accumulated angle
     */
    static float get_accumulated_angle(motor_id_t motor);

private:

    static action_t action;

    class VisionThread : public chibios_rt::BaseStaticThread<256> {
    public:

        bool started = false;

    private:

        time_msecs_t last_apply_time = 0;

        static constexpr unsigned VISION_FEED_INTERVAL = 5;  // thread interval [ms]

        void main() final;
    };

    static VisionThread visionThread;
    static chibios_rt::ThreadReference visionThreadReference;

};

#endif //META_INFANTRY_GIMBAL_LOGIC_H

/** @} */