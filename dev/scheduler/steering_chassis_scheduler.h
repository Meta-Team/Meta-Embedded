//
// Created by Wu Feiyang on 2023/5/8.
//

#ifndef META_EMBEDDED_STEERING_CHASSIS_SCHEDULER_H
#define META_EMBEDDED_STEERING_CHASSIS_SCHEDULER_H

#include "can_motor_controller.h"
#include "scheduler_base.h"
#include <cmath>
#define CHASSIS_USE_STEER

class SteerChassisSKD: public SKDBase{
public:

    static enum install_mode_t{
        POSITIVE =  1,
        NEGATIVE = -1,
    } install_mode;
    /**
    * Initialize this module
    * @param skd_prio priority for scheduler thread
    * @param wheel_base            Distance between front axle and the back axle [mm]
    * @param wheel_dist_from_center          Radius of four steering wheels position from chassis center [mm]
    * @param wheel_circumference   Circumference of wheels [mm]
    * @param gimbal_offset offset from gimbal
    */
    static void init(tprio_t skd_prio, float wheel_base, float wheel_dist_from_center, float wheel_circumference, float gimbal_offset = 0);

    /**
     * Set the mode for chassis.
     * @param mode_
     */
    static void set_mode(mode_t mode_);

    /**
     * Set target values
     * @param vx     Target velocity along the x axis (right) with respect to gimbal coordinate [mm/s]
     * @param vy     Target velocity along the y axis (up) with respect to gimbal coordinate [mm/s]
     * @param omega  Target angular velocity [degree/s]
     */
    static void set_target(float vx, float vy, float omega);

    static float w_to_v_ratio;

    static float v_to_wheel_angular_velocity;

private:

    /**
     * @brief Thread for performing velocity decompose and angle transformation.
     */
    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned SKD_INTERVAL = 5; //[ms]
        void main() final;
    };

    static SKDThread skd_thread;

    /// Gimbal Behavior Control
    static mode_t mode;

/// TODO: Re-Enable These functions.
#ifdef ENABLE_CHASSIS_SHELL
    static bool motor_enabled;
    static DECL_SHELL_CMD(cmdInfo);
    static DECL_SHELL_CMD(cmdEnableFeedback);
    static DECL_SHELL_CMD(cmdPID);
    static DECL_SHELL_CMD(cmdEnableMotor);
#endif

};

#endif //META_EMBEDDED_STEERING_CHASSIS_SCHEDULER_H
