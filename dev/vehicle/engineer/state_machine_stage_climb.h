//
// Created by liuzikai on 2019-02-24.
//

#ifndef META_INFANTRY_STATE_MACHINE_STAGE_CLIMB_H
#define META_INFANTRY_STATE_MACHINE_STAGE_CLIMB_H

#include "ch.hpp"
#include "hal.h"

#include "serial_shell.h"

#include "elevator.h"

#include "thread_chassis.hpp"
#include "thread_elevator.hpp"

#define STAGE_CLIMB_STATE_MACHINE_ENABLE_TIMEOUT  FALSE

#define STAGE_CLIMB_STATE_MACHINE_WORKING_AREA_SIZE 2048

class StageClimbStateMachine : public chibios_rt::BaseStaticThread<STAGE_CLIMB_STATE_MACHINE_WORKING_AREA_SIZE> {

public:

    StageClimbStateMachine(ChassisThread& chassisThread_, ElevatorThread& elevatorThread_)
    : chassisThread(chassisThread_), elevatorThread(elevatorThread_) {};

    enum action_t {
        STOP,
        UPWARD, // moving up to the stage
        DOWNWARD  // moving down to the stage
    };

    /**
     * Get get the current action of the state machine
     * @return
     */
    action_t get_current_action();

    /**
     * Start the thread to perform upward action
     * @param prio priority of the thread
     * @return whether the thread starts successfully
     */
    bool start_up_action(tprio_t prio);

    /**
     * Start the thread to perform downward action
     * @param prio priority of the thread
     * @return whether the thread starts successfully
     */
    bool start_down_action(tprio_t prio);


private:

    ChassisThread& chassisThread;
    ElevatorThread& elevatorThread;

    action_t action_ = STOP;

    void main() final;

    chibios_rt::ThreadReference start(tprio_t) final {return nullptr;}; // delete this function

    static constexpr int ELEVATOR_CHECK_INTERVAL = 5; // [ms]
    static constexpr float STAGE_HEIGHT = 21.0f;

};


#endif //META_INFANTRY_STATE_MACHINE_STAGE_CLIMB_H
