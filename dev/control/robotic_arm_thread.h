//
// Created by liuzikai on 2019-02-24.
//

#ifndef META_INFANTRY_ROBOTIC_ARM_THREAD_H
#define META_INFANTRY_ROBOTIC_ARM_THREAD_H

#include "ch.hpp"
#include "hal.h"
#include "common_macro.h"
#include "robotic_arm.h"

#define ROBOTIC_ARM_THREAD_WORKING_AREA_SIZE 512

class RoboticArmThread : public chibios_rt::BaseStaticThread<ROBOTIC_ARM_THREAD_WORKING_AREA_SIZE> {
    enum status_t {
        STOP,
        ACTIONING
    };

    status_t get_status();

    bool start_actions(tprio_t prio);

    void emergency_stop();

private:

    status_t status = STOP;

    void main() final;

private:

    /** Configurations **/
    static constexpr int rotation_motor_action_interval = 20; // [ms]

    // TODO: fill these parameters
    
    static constexpr uint16_t rotation_motor_inside_angle_raw = 0;
    static constexpr uint16_t rotation_motor_outside_angle_raw = 0;

    static constexpr uint16_t rotation_motor_outward_trigger_angle_raw = 0;
    static constexpr uint16_t rotation_motor_inward_trigger_angle_raw = 0;
};


#endif //META_INFANTRY_ROBOTIC_ARM_THREAD_H
