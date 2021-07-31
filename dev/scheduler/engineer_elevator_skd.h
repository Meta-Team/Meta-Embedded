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

#define ELEVATOR_CLOSE_LOOP_CONTROL TRUE

/**
 * @note positive height - chassis lift up
 */
class EngineerElevatorSKD {

public:

    static void start(tprio_t thread_prio, float time_2_height_ratio_);

    static void set_target_height(float target_height_);

    static void set_target_movement(float target_location_);

    enum homing_direction_t {
        VERTICAL,
        HORIZONTAL
    };

#if ELEVATOR_CLOSE_LOOP_CONTROL
    static void homing(homing_direction_t homing_direction_);
#endif
private:

    static enum operation_t {
        UPWARD,
        DOWNWARD,
        STOP
    } operation;
    // For vertical, FORWARD means upward, BACKWARD means downward.
    // For horizontal, the name conforms with the description.

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]
        void main() final;
    };

    static SKDThread skdThread;

    friend void cmd_elevator_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]);

    friend void cmd_elevator_set_target_velocities(BaseSequentialStream *chp, int argc, char *argv[]);

    static float target_height, stop_judge_threshold, time_2_length_ratio, current_height;
};


#endif //META_INFANTRY_ENGINEER_ELEVATOR_SKD_H
