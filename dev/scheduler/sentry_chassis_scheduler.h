//
// Created by zhukerui on 2019/4/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H

#include "ch.hpp"
#include "hal.h"

#include "sentry_chassis_interface.h"
#include "pid_controller.hpp"


class SChassisSKD : public SChassisBase, public PIDControllerBase {
public:

    enum mode_t {
        FORCED_RELAX_MODE,   // zero force (Still taking control of ChassisIF. External writing to target currents
                             // will leads to conflicts.)
        ABS_DEST_MODE,       // absolute position on the rail
        PARAM_ADJUST_MODE    // TODO: make compatible for pa program
    };

    enum install_direction_t {
        POSITIVE = 1,
        NEGATIVE = -1
    };


    static void start(install_direction_t left_motor_install, install_direction_t right_motor_install,
                      tprio_t thread_prio);

    static void load_pid_params(pid_params_t sentry_a2v_params, pid_params_t sentry_v2i_params);

    static void reset_origin();

    static void set_mode(mode_t target_mode);

    /**
     * @brief set the target position, bigger position will drive the chassis towards right
     * @param dist the target position
     */
    static void set_destination(float dist);

    static float present_position();
    static float present_velocity();

private:

    // Local storage
    static mode_t mode;
    static install_direction_t motor_install_[2];

    static PIDController a2v_pid;
    static PIDController v2i_pid[MOTOR_COUNT];

    static float target_position;  // [cm], bigger position will drive the chassis towards right
    static float target_velocity;  // [cm/s], calculated target velocity, middle value
    static int target_current[MOTOR_COUNT];  // local storage


    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]
        void main() final;
    };

    static SKDThread skdThread;

};


#endif //META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H