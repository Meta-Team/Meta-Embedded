//
// Created by Qian Chen on 3/27/21.
//

#ifndef META_INFANTRY_ENGINEER_GRAB_SKD_H
#define META_INFANTRY_ENGINEER_GRAB_SKD_H

#include "ch.hpp"

#include "engineer_grab_mech_interface.h"
#include "pid_controller.hpp"

class engineerGrabSKD : public EngGrabMechBase, public PIDControllerBase {
public:

    enum mode_t{
        FORCE_RELAX_MODE,
        NORMAL_MODE
    };

    enum install_direction {
        POSITIVE = 1,
        NEGATIVE = -1
    };

    /***
     * @brief Start SKDThread.
     * @param SKDThreadPRIO priority for SKDThread.
     * ***/
    static void start(tprio_t SKDThreadPRIO, install_direction direction_[2]);

    /***
     * @brief Load PID parameters for motors of translation belt.
     * @param pidParams PID parameters (2 sets) for translation belt.
     * ***/
    static void load_pid_params(PIDController::pid_params_t pidParams[2]);

    /***
     * @brief Set target velocity for translation belt.
     * @param targetVelocity_ the target velocity for belt.
     * ***/
     static void set_target_velocity(float targetVelocity_);
private:

    static float targetVelocity;

    static install_direction direction[2];

    static PIDController v2iController[2];

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };
    static SKDThread skdThread;

    static constexpr unsigned int SKD_THREAD_INTERVAL = 1;
};


#endif //META_INFANTRY_ENGINEER_GRAB_SKD_H
