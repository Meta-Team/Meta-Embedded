//
// Created by Quoke on 3/7/2022.
// Modified by Wu Feiyang on 5/8/2023
//

#ifndef META_INFANTRY_SCHEDULER_BASE_H
#define META_INFANTRY_SCHEDULER_BASE_H

/**
 * @brief Scheduler base for vehicles with gimbal and chassis.
 */
class SKDBase {
public:
    enum mode_t {
        FORCED_RELAX_MODE,  ///< zero force (but still taking control of GimbalIF)
        CHASSIS_REF_MODE,   ///< Gimbal angle are in chassis frame
#if ENABLE_AHRS
        GIMBAL_REF_MODE     ///< Gimbal angle are in gimbal frame
#endif
    };

    /**
     * @brief Input target velocity in X axis (orthogonal to gimbal vehicle's direction)
     */
    static float target_vx;

    /**
     * @brief Input target velocity in X axis (parallel to gimbal vehicle's direction)
     */
    static float target_vy;

    /**
     * @brief Input target chassis angular velocity.
     */
    static float target_omega;

    /**
     * @brief The distance of chassis and gimbal's geometric centers.
     */
    static float chassis_gimbal_offset_;

};


#endif //META_INFANTRY_SCHEDULER_BASE_H
