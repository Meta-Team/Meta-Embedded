//
// Created by liuzikai on 2019-05-01.
//

#ifndef META_INFANTRY_SHOOT_SCHEDULER_H
#define META_INFANTRY_SHOOT_SCHEDULER_H

#include "ch.hpp"

#include "gimbal_interface.h"
#include "pid_controller.hpp"


class ShootSKD : public GimbalBase, public PIDControllerBase {

public:

    enum mode_t {
        STOP_MODE,                // zero force
        // RELATIVE_ANGLE_MODE,   // target_angle of yaw is relative to chassis
                ABS_ANGLE_MODE,           // target_angle of yaw is relative to ground
        PARAM_ADJUST_MODE         // for PID parameter adjustment program
    };

    /**
     * Initialize the shooter controller
     * @param degree_per_bullet
     * @param bullet_loader_v2i_params
    */

    static void start(float degree_per_bullet, tprio_t thread_prio);

    /**
     * Change PID parameters of bullet loader
     * @param bullet_loader_v2i_params
     */

    static void load_pid_params(PIDControllerBase::pid_params_t bullet_loader_v2i_params);

    /**
     * Perform calculation from velocity to current and put result into target_current[]
     * @param bullet_per_second   shoot speed
     */
    static void set_buller_loader(float bullet_per_second);

    /**
     * Set friction wheel duty cycle
     * @param duty_cycle from 0 to 1
     */
    static void set_friction_wheels(float duty_cycle);



private:

    static float degree_per_bullet_;

    static PIDController v2i_pid;

    static constexpr unsigned int SKD_THREAD_INTERVAL = 1; // PID calculation interval [ms]

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skdThread;


    friend class ShootDebugThread;
};


#endif //META_INFANTRY_SHOOT_SCHEDULER_H
