//
// Created by liuzikai on 2019-02-24.
//

#ifndef META_INFANTRY_ELEVATOR_CONTROLLER_H
#define META_INFANTRY_ELEVATOR_CONTROLLER_H

#include "ch.hpp"
#include "hal.h"

class ElevatorController {
public:

    enum status_t {
        STOP,

        UP_LOWER_ALL_WHEELS,
        UP_MOVE_FORWARD_1,
        UP_LIFT_FRONT_WHEELS,
        UP_MOVE_FORWARD_2,
        UP_LIFT_REAR_WHEELS,
        UP_MOVE_FORWARD_3,

        DOWN_LOWER_REAR_WHEELS,
        DOWN_MOVE_BACKWARD_1,
        DOWN_LOWER_FRONT_WHEELS,
        DOWN_MOVE_BACKWARD_2,
        DOWN_LIFT_ALL_WHEELS,
    };

    static status_t get_status();

    static float get_front_target_position();
    static float get_rear_target_position();

    static void lift_all_wheels();
    static void lift_front_wheels();
    static void lift_rear_wheels();
    static void lower_all_wheels();
    static void lower_front_wheels();
    static void lower_rear_wheels();

    /**
     * @brief start the status machine to perform a full up action
     * @param thread_prio
     * @note elevator thread will TAKE OVER control of chassis motor, so stop other threads that control chassis
     */
    static bool perform_up_action(tprio_t thread_prio);
    static bool perform_down_action(tprio_t thread_prio);

private:

    static status_t status;
    static float front_target_position;
    static float rear_target_position;

private:

    /** Configurations **/
    static constexpr float stage_height = 20; // [cm]

};


#endif //META_INFANTRY_ELEVATOR_CONTROLLER_H
