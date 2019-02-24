//
// Created by liuzikai on 2019-02-24.
//

#ifndef META_INFANTRY_ELEVATOR_CONTROLLER_H
#define META_INFANTRY_ELEVATOR_CONTROLLER_H

#include "ch.hpp"
#include "hal.h"

#include "elevator_interface.h"

#include "chassis_common.h"
#include "chassis_interface.h"
#include "chassis_controller.h"

#define ELEVATOR_THREAD_WORKING_AREA_SIZE 512

/**
 * @name ElevatorThread
 * @pre ElevatorInterface is init() with proper CAN interface
 * @usage 1. start_up_actions() or start_down_actions() with specific thread priority
 *        2. periodically read get_chassis_target_vy() to another thread for chassis motor
 */
class ElevatorThread : public chibios_rt::BaseStaticThread<ELEVATOR_THREAD_WORKING_AREA_SIZE> {
public:

    enum status_t {
        STOP,
        UPWARD, // moving up to the stage
        DOWNWARD  // moving down to the stage
    };

    status_t get_status();

    bool start_up_actions(tprio_t prio);
    bool start_down_actions(tprio_t prio);

    void emergency_stop();

    float get_chassis_target_vy();

private:

    status_t status_ = STOP;
    float chassis_target_vy_ = 0;

    void main() final;

    chibios_rt::ThreadReference start(tprio_t prio) final {return nullptr;}; // delete this function

private:

    /** Configurations **/

    static constexpr float stage_height_ = 20; // [cm]

    static constexpr int elevator_check_interval_ = 20; // [ms]

    // NOTICE: to allow the PID params works well, chassis_thread_interval should be the same of thread of chassis
    static constexpr int chassis_action_interval_ = 20; // [ms]

};


#endif //META_INFANTRY_ELEVATOR_CONTROLLER_H
