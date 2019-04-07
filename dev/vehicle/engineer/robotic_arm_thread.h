//
// Created by liuzikai on 2019-02-24.
//

/**
 * This file contain a thread for Engineer robotic arm. This thread is started manually when there are actions needed
 * to be perform, and ends as the actions are done.
 */

#ifndef META_INFANTRY_ROBOTIC_ARM_THREAD_H
#define META_INFANTRY_ROBOTIC_ARM_THREAD_H

#include "ch.hpp"
#include "hal.h"
#include "common_macro.h"
#include "robotic_arm.h"

#define ROBOTIC_ARM_THREAD_WORKING_AREA_SIZE 512

#if defined(BOARD_RM_2018_A)
#else
#error "RoboticArmThread is only developed for RM board 2018 A."
#endif

/**
 * @name RoboticArmThread
 * @brief a thread to control robotic arm to complete a cycle of fetching bullet
 * @pre RoboticArm is properly init() and reset_front_angle()
 * @usage 0. instantiate an object
 *        1. start_actions() with specific thread priority to perform the fetching actions
 */
class RoboticArmThread : public chibios_rt::BaseStaticThread<ROBOTIC_ARM_THREAD_WORKING_AREA_SIZE> {
public:
    enum status_t {
        STOP,
        INITIAL_OUTWARD,
        FETCH_ONCE,
        FINAL_INWARD
    };

    /**
     * @brief get the status of this thread
     * @return
     */
    status_t get_status();

    /**
     * @brief start a series of actions to make the robotic arm outward
     * @param prio thread priority
     * @return whether the actions can be start.
     */
    bool start_initial_outward(tprio_t prio);

    bool start_one_fetch(tprio_t prio);

    bool start_final_inward(tprio_t prio);

    void emergency_stop();
    
    bool is_outward();

private:

    status_t status_ = STOP;
    bool should_stop_ = false;

    void main() final;

    chibios_rt::ThreadReference start(tprio_t) final {return nullptr;}; // delete this function

private:

    /** Configurations **/

    static constexpr int motor_action_interval = 20; // [ms]

    // Outward movement results in negative angle

    // Measured data: startup = 0, middle upward = -66, outside = -160

    // If actual angle is GREATER than motor_inside_target_angle, we think that the robotic arm is completely inside
    static constexpr float motor_inside_target_angle = -3; // [degree]
};


#endif //META_INFANTRY_ROBOTIC_ARM_THREAD_H
