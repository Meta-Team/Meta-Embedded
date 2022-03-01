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
#include "lidar_interface.h"
#include "vision_scheduler.h"
#include "trajectory_calculator.hpp"

/**
 * @name GimbalLG
 * @note LG stands for "logic"
 * @brief By-passing logic-level module to control GimbalLG
 * @pre GimbalSKD has started properly
 * @usage Invoke set_action() and set_target_angle() to control gimbal
 */
using namespace chibios_rt;
class GimbalLG {
public:
    enum mode_t {
        FORCED_RELAX_MODE,  // zero force (but still taking control of GimbalIF)
        CHASSIS_REF_MODE,   //
        GIMBAL_REF_MODE // TODO: Use MIMO controller to optimize this mode
    };

    static mode_t mode;
    /**
     * Initialize the gimbal Logic
     * @param vision_prio
     * @param gimbal_control_prio
     */
    static void init (tprio_t vision_prio, tprio_t gimbal_control_prio);

    static void set_target_angle(float yaw_target_angle, float pitch_target_angle);

    static void set_action(mode_t mode_);
#if defined(ENABLE_SUBPITCH)
    class GimbalSubPitchThd : public BaseStaticThread<512>{
        time_msecs_t INTERVAL = 5;
        void main() final;
    };
#endif
};

#endif //META_INFANTRY_GIMBAL_LOGIC_H

/** @} */