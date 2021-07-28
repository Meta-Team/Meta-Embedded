//
// Created by Kerui Zhu on 7/9/2019.
//

#ifndef META_INFANTRY_ENGINEER_ELEVATOR_SKD_H
#define META_INFANTRY_ENGINEER_ELEVATOR_SKD_H

#include "ch.hpp"
#include "hal.h"
#include "common_macro.h"

#include "engineer_elevator_interface.h"
#include "pid_controller.hpp"


/**
 * @note positive height - chassis lift up
 */
class EngineerElevatorSKD {

public:

    static void start(tprio_t thread_prio, float time_2_height_ratio_);

    static void set_target_height(float target_height_);

    static void set_target_movement(float target_location_);

    static float get_current_height();

    static float get_current_movement();

private:

    static enum operation_t {
        FORWARD,
        BACKWARD,
        STOP
    } vertical_operation, horizontal_operation;

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]
        void main() final;
    };

    static SKDThread skdThread;

    friend void cmd_elevator_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void cmd_elevator_set_target_velocities(BaseSequentialStream *chp, int argc, char *argv[]);

    static float time_2_length_ratio;

    static float current_height, target_height, current_location, target_location, stop_judge_threshold;
};


#endif //META_INFANTRY_ENGINEER_ELEVATOR_SKD_H
