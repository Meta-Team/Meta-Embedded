//
// Created by liuzikai on 2019-02-24.
//

/**
 * This file contain a thread for Engineer elevator. This thread is started manually when there are actions needed to be
 * perform, and ends as the actions are done.
 */

#ifndef META_INFANTRY_ELEVATOR_THREAD_H
#define META_INFANTRY_ELEVATOR_THREAD_H

#include "ch.hpp"
#include "hal.h"

#include "elevator_interface.h"

#define ELEVATOR_THREAD_WORKING_AREA_SIZE 2014

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

    /**
     * Get status of elevator thread
     * @return
     */
    status_t get_status();

    /**
     * Start the thread to perform upward action
     * @param prio priority of the thread
     * @return whether the thread starts successfully
     */
    bool start_up_actions(tprio_t prio);

    /**
     * Start the thread to perform downward action
     * @param prio priority of the thread
     * @return whether the thread starts successfully
     */
    bool start_down_actions(tprio_t prio);

    /**
     * Stop the thread
     */
    void emergency_stop();

    /**
     * Get target chassis velocity forward (vx). Used by the ChassisThread
     * @return target_vx
     */
    float get_chassis_target_vy();

private:

    status_t status_ = STOP;
    float chassis_target_vy_ = 0;

    void main() final;

    chibios_rt::ThreadReference start(tprio_t) final {return nullptr;}; // delete this function

private:

    /** Configurations **/

    static constexpr float stage_height_ = 20; // height of the stage [cm]

    static constexpr int elevator_check_interval_ = 20; // [ms]

    // NOTICE: to allow the PID params works well, chassis_thread_interval should be the same of thread of chassis
    static constexpr int chassis_action_interval_ = 20; // [ms]

};


#endif //META_INFANTRY_ELEVATOR_THREAD_H
