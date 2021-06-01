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
     * @param pidParams PID parameters (4 sets) for translation belt [0-1] and grabber [2-3].
     * ***/
    static void load_v2i_pid_params(pid_params_t *pidParams);

    /***
 * @brief Load PID parameters for motors of translation belt.
 * @param pidParams PID parameters (2 sets) for grabber.
 * ***/
    static void load_a2v_pid_params(pid_params_t *pidParams);

    /***
     * @brief Set target velocity for translation belt.
     * @param targetVelocity_ the target velocity for belt.
     * ***/
     static void set_belt_target_velocity(float targetVelocity_);

     static void invoke_rising();

     static void invoke_lowering();

     static int echo_status();
private:

    static float target_velocity[MOTOR_COUNT];
    static float target_angle[MOTOR_COUNT];
    static PIDController a2vController[MOTOR_COUNT];
    static PIDController v2iController[MOTOR_COUNT];

//    static float beltTargetVelocity;

//    static float grabberTargetVelocity[2];

    static install_direction direction[MOTOR_COUNT];

//    static PIDController v2iController[4]; // [0-1, belt, 2-3, grabber 2006]

//    static PIDController a2vController[2]; // [0-1, grabber 2006].

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };
    static SKDThread skdThread;

    static constexpr unsigned int SKD_THREAD_INTERVAL = 1;

    enum arm_status_t{
        RISING,
        RISE_IDLE,
        RISED,
        LOWERING,
        LOWER_IDLE,
        LOWERED
    };

    static arm_status_t armStatus;
};


#endif //META_INFANTRY_ENGINEER_GRAB_SKD_H
