//
// Created by kerui on 2021/7/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_SCHEDULER_H
#define META_INFANTRY_SENTRY_CHASSIS_SCHEDULER_H

#include "new_sentry_chassis_interface.h"

#include "pid_controller.hpp"

class SChassisSKD : public SChassisBase, public PIDControllerBase {

public:
    enum mode_t {
        FORCED_RELAX_MODE,
        ENABLED
    };

    static void start(tprio_t thread_prio);

    static void load_pid_params(pid_params_t a2v_pid_params, pid_params_t v2i_pid_params);

    static void set_mode(mode_t skd_mode);

    static void set_target(float target_location);

    static float get_location(motor_id_t id) { return accumulated_displacement[id]; };

    static void cmdFeedback(void *);
    static const Shell::Command shellCommands[];

private:

    static mode_t mode;
    static float target_location_;
    static float accumulated_displacement[MOTOR_COUNT];
    static float actual_velocity[MOTOR_COUNT];
    static float target_velocity[MOTOR_COUNT];
    static int target_current[MOTOR_COUNT];
    static PIDController a2v_pid[MOTOR_COUNT];
    static PIDController v2i_pid[MOTOR_COUNT];

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skd_thread;

    static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]

    static bool motor_enabled;
    static DECL_SHELL_CMD(cmdInfo);
    static DECL_SHELL_CMD(cmdEnableFeedback);
    static DECL_SHELL_CMD(cmdPID);
    static DECL_SHELL_CMD(cmdEnableMotor);

    static float constexpr DISPLACEMENT_PER_ROUND = 17.28f;
};


#endif //META_INFANTRY_SENTRY_CHASSIS_SCHEDULER_H
