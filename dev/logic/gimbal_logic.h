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

    enum action_t {
        FORCED_RELAX_MODE,
        ABS_ANGLE_MODE
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
     * Set target angles
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

};

#endif //META_INFANTRY_GIMBAL_LOGIC_H

/** @} */