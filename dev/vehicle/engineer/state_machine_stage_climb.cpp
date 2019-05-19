//
// Created by liuzikai on 2019-02-24.
//

#include "state_machine_stage_climb.h"


StageClimbStateMachine::action_t StageClimbStateMachine::get_current_action() {
    return action_;
}

bool StageClimbStateMachine::start_up_action(tprio_t prio) {
    if (action_ != STOP) return false;
    action_ = UPWARD;
    chibios_rt::BaseStaticThread<STAGE_CLIMB_STATE_MACHINE_WORKING_AREA_SIZE>::start(prio);
    return true;
}

bool StageClimbStateMachine::start_down_action(tprio_t prio) {
    if (action_ != STOP) return false;
    action_ = DOWNWARD;
    chibios_rt::BaseStaticThread<STAGE_CLIMB_STATE_MACHINE_WORKING_AREA_SIZE>::start(prio);
    return true;
}

void StageClimbStateMachine::main() {

    setName("elevator");

    time_msecs_t t;

    if (action_ == STOP) {
        exit(-1);
    } else if (action_ == UPWARD) {

        chassisThread.enable_external_mode();
        chassisThread.set_external_target(0, 0, 0);

        LOG("[ELE UP] Start");

        /** Step 1. Lift the vehicle **/
        LOG("[ELE UP] Step 1...");

        sleep(TIME_MS2I(1000));
        elevatorThread.set_front_target_height(STAGE_HEIGHT);
        elevatorThread.set_back_target_height(STAGE_HEIGHT);
        sleep(TIME_MS2I(5000));
        t = SYSTIME;
        while (!Elevator::motor_reach_target(Elevator::FR) ||
               !Elevator::motor_reach_target(Elevator::FL) ||
               !Elevator::motor_reach_target(Elevator::BL) ||
               !Elevator::motor_reach_target(Elevator::BR)) {
#if STAGE_CLIMB_STATE_MACHINE_ENABLE_TIMEOUT
            if (SYSTIME - t > 20000) {
                LOG_ERR("[ELE UP] Step 1 timeout, is_in_action: %d %d %d %d",
                        Elevator::motor_reach_target(Elevator::FR),
                        Elevator::motor_reach_target(Elevator::FL),
                        Elevator::motor_reach_target(Elevator::BL),
                        Elevator::motor_reach_target(Elevator::BR));
                break;
            }
#endif
            sleep(TIME_MS2I(ELEVATOR_CHECK_INTERVAL));
        }

        sleep(TIME_MS2I(100)); // Time interval between steps

        /** Step 2. Move forward to make the front assistant wheels to get onto the stage **/
        LOG("[ELE UP] Step 2...");

        chassisThread.set_external_target(0, -500, 0);
        sleep(TIME_MS2I(500));

        chassisThread.set_external_target(0, 0, 0);
        sleep(TIME_MS2I(100));

        /** Step 3. Lift the front wheels **/
        LOG("[ELE UP] Step 3...");

        elevatorThread.set_front_target_height(0);
        sleep(TIME_MS2I(5000));
        t = SYSTIME;
        while (!Elevator::motor_reach_target(Elevator::FR) ||
               !Elevator::motor_reach_target(Elevator::FL)) {
#if STAGE_CLIMB_STATE_MACHINE_ENABLE_TIMEOUT
            if (SYSTIME - t > 20000) {
                LOG_ERR("[ELE UP] Step 3 timeout, is_in_action: %d %d %d %d",
                        Elevator::motor_reach_target(Elevator::FR),
                        Elevator::motor_reach_target(Elevator::FL),
                        Elevator::motor_reach_target(Elevator::BL),
                        Elevator::motor_reach_target(Elevator::BR));
                break;
            }
#endif
            sleep(TIME_MS2I(ELEVATOR_CHECK_INTERVAL));
        }

        sleep(TIME_MS2I(500));  // Time interval between steps

        /** Step 4. Move forward to make the front wheels and rear assistant wheel to be on the stage **/
        LOG("[ELE UP] Step 4...");
        sleep(TIME_MS2I(800));
        chassisThread.set_external_target(0, -1900, 0);
        sleep(TIME_MS2I(500));
        chassisThread.set_external_target(0, -400, 0);
        sleep(TIME_MS2I(1000));
        chassisThread.set_external_target(0, -200, 0);
        sleep(TIME_MS2I(500));
        chassisThread.set_external_target(0, 0, 0);

        sleep(TIME_MS2I(100));  // Time interval between steps

        /** Step 5. Lift the rear wheels **/
        LOG("[ELE UP] Step 5...");

        elevatorThread.set_back_target_height(0);
        sleep(TIME_MS2I(5000));
        t = SYSTIME;
        while (!Elevator::motor_reach_target(Elevator::BL) ||
               !Elevator::motor_reach_target(Elevator::BR)) {
#if STAGE_CLIMB_STATE_MACHINE_ENABLE_TIMEOUT
            if (SYSTIME - t > 20000) {
                LOG_ERR("[ELE UP] Step 5 timeout, is_in_action: %d %d %d %d",
                        Elevator::motor_reach_target(Elevator::FR),
                        Elevator::motor_reach_target(Elevator::FL),
                        Elevator::motor_reach_target(Elevator::BL),
                        Elevator::motor_reach_target(Elevator::BR));
                break;
            }
#endif
            Elevator::send_elevator_currents();
            sleep(TIME_MS2I(ELEVATOR_CHECK_INTERVAL));
        }

        /** Step 6. Move forward to make the rear wheels to be on the stage **/

        chassisThread.set_external_target(0, -1800, 0);
        sleep(TIME_MS2I(1000));
        chassisThread.set_external_target(0, -500, 0);
        sleep(TIME_MS2I(400));
        chassisThread.set_external_target(0, 0, 0);

        /** Complete **/

        chassisThread.disable_external_mode();

        LOG("[ELE UP] Complete");
        action_ = STOP;
        exit(0);
    } else if (action_ == DOWNWARD) {

    }
    // TODO: write action of going downward
}

