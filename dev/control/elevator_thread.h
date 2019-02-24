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

class ElevatorThread : public chibios_rt::BaseStaticThread<ELEVATOR_THREAD_WORKING_AREA_SIZE> {
public:

    enum status_t {
        STOP,
        UPWARD,
        DOWNWARD
    };

    status_t get_status();

    bool start_up_actions(tprio_t prio);
    bool start_down_actions(tprio_t prio);

    void emergency_stop();

    float chassis_target_vx = 0;

private:

    status_t status = STOP;

    void main() final;

private:

    /** Configurations **/
    static constexpr float stage_height = 20; // [cm]
    static constexpr int elevator_check_interval = 20; // [ms]
    static constexpr int chassis_action_interval = 20; // [ms]
    // TODO: to allow the PID params works well, chassis_thread_interval should be the same of thread of chassis

};


#endif //META_INFANTRY_ELEVATOR_CONTROLLER_H
