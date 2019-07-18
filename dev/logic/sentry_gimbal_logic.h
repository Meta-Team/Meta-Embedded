//
// Created by Kerui Zhu on 7/17/2019.
//

#ifndef META_INFANTRY_SENTRY_GIMBAL_LOGIC_H
#define META_INFANTRY_SENTRY_GIMBAL_LOGIC_H

#include "gimbal_scheduler.h"
#include "vision_port.h"

class SGimbalLG: public GimbalBase {

public:

    static void init();

    enum action_t {
        FORCED_RELAX_MODE,
        ABS_ANGLE_MODE
    };

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

};


#endif //META_INFANTRY_SENTRY_GIMBAL_LOGIC_H
