//
// Created by Quoke on 8/2/2022.
//

#ifndef META_EMBEDDED_SENTRY_CHASSIS_LOGIC_H
#define META_EMBEDDED_SENTRY_CHASSIS_LOGIC_H

#include "sentry_chassis_scheduler.h"
#include "random"

class SChassisLG {
public:

    enum mode_t {
        FORCED_RELAX_MODE, ///< Stop immediately and then cut off power.
        AUTO_MODE,         ///< Auto move with randomly picked positions.
        MANUAL_MODE        ///< Controlled by remote controller.
    };

    /**
     * @brief Initialize sentry chassis logic.
     * @param mtn_ctl_prio
     */
    static void init(tprio_t mtn_ctl_prio);

    /**
     * @brief Set mode for sentry chassis logic.
     * @param mode [in] Mode of sentry chassis. Available options: FORCED_RELAX_MODE,
     *                  AUTO_MODE, MANUAL_MODE.
     */
    static void set_mode(mode_t mode);

    /**
     * @brief Set velocity for sentry chassis. Only available in MANUAL_MODE.
     * @param target_velocity
     */
    static void set_velocity(float target_velocity);

private:

    static mode_t mode;
    static float target_velocity;

    class MotionControlThread : public BaseStaticThread<512> {
        void main() final;
        constexpr static unsigned SCHASSIS_MTN_CTL_INTERVAL = 2; // [ms]
    };

    static MotionControlThread motion_control_thread;
};


#endif //META_EMBEDDED_SENTRY_CHASSIS_LOGIC_H
