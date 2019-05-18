//
// Created by liuzikai on 2019-02-24.
//

#include "state_machine_stage_climb.h"
#include "buzzer.h"


StageClimbThread::status_t StageClimbThread::get_status() {
    return status_;
}

bool StageClimbThread::start_up_actions(tprio_t prio) {
    if (status_ != STOP) return false;
    status_ = UPWARD;
    chibios_rt::BaseStaticThread<ELEVATOR_THREAD_WORKING_AREA_SIZE>::start(prio);
    return true;
}

bool StageClimbThread::start_down_actions(tprio_t prio) {
    if (status_ != STOP) return false;
    status_ = DOWNWARD;
    chibios_rt::BaseStaticThread<ELEVATOR_THREAD_WORKING_AREA_SIZE>::start(prio);
    return true;
}

void StageClimbThread::emergency_stop() {
    status_ = STOP;
    exit(0xFF); // FIXME: this doesn't work since it terminate the call thread
}

float StageClimbThread::get_chassis_target_vy() {
    return chassis_target_vy_;
}

void StageClimbThread::main() {

    setName("elevator");

    systime_t t;

    if (status_ == STOP) {
        exit(-1);
    } else if (status_ == UPWARD) {

        LOG("[ELE UP] Start");

        /** Step 1. Lift the vehicle **/
        LOG("[ELE UP] Step 1...");

        sleep(TIME_MS2I(1000));
        Elevator::calc_front(-Elevator::STAGE_HEIGHT);
        Elevator::calc_back(-Elevator::STAGE_HEIGHT);

        t = chVTGetSystemTime();
        while (Elevator::feedback[0].in_action ||
               Elevator::feedback[1].in_action ||
               Elevator::feedback[2].in_action ||
               Elevator::feedback[3].in_action) {

            if (TIME_I2MS(chVTGetSystemTime() - t) > 14000) {
                LOG_ERR("[ELE UP] Step 1 timeout, is_in_action: %d %d %d %d",
                        Elevator::feedback[0].in_action,
                        Elevator::feedback[1].in_action,
                        Elevator::feedback[2].in_action,
                        Elevator::feedback[3].in_action);
                break;
            }

            Elevator::send_elevator_currents();
            sleep(TIME_MS2I(elevator_check_interval_));
        }

        sleep(TIME_MS2I(100)); // Time interval between steps

        /** Step 2. Move forward to make the front assistant wheels to get onto the stage **/
        LOG("[ELE UP] Step 2...");

        chassis_target_vy_ = -500;
        sleep(TIME_MS2I(500));

        chassis_target_vy_ = 0;
        sleep(TIME_MS2I(100));

        /** Step 3. Lift the front wheels **/
        LOG("[ELE UP] Step 3...");

        Elevator::calc_front(0);

        t = chVTGetSystemTime();
        while (Elevator::feedback[Elevator::FR].in_action ||
               Elevator::feedback[Elevator::FL].in_action) {
            if (TIME_I2MS(chVTGetSystemTime() - t) > 14000) {
                LOG_ERR("[ELE UP] Step 3 timeout, is_in_action: %d %d %d %d",
                        Elevator::feedback[0].in_action,
                        Elevator::feedback[1].in_action,
                        Elevator::feedback[2].in_action,
                        Elevator::feedback[3].in_action);
                break;
            }

            Elevator::send_elevator_currents();
            sleep(TIME_MS2I(elevator_check_interval_));
        }

        sleep(TIME_MS2I(500));  // Time interval between steps

        /** Step 4. Move forward to make the front wheels and rear assistant wheel to be on the stage **/
        LOG("[ELE UP] Step 4...");
        sleep(TIME_MS2I(800));
        chassis_target_vy_ = -2400;
        sleep(TIME_MS2I(1000));
        chassis_target_vy_ = -500;
        sleep(TIME_MS2I(1500));
        chassis_target_vy_ = 0;

        sleep(TIME_MS2I(100));  // Time interval between steps

        /** Step 5. Lift the rear wheels **/
        LOG("[ELE UP] Step 5...");

        Elevator::calc_back(0);

        t = chVTGetSystemTime();
        while (Elevator::feedback[Elevator::BL].in_action ||
               Elevator::feedback[Elevator::BR].in_action) {
            if (TIME_I2MS(chVTGetSystemTime() - t) > 15000) {
                LOG_ERR("[ELE UP] Step 3 timeout, is_in_action: %d %d %d %d",
                        Elevator::feedback[0].in_action,
                        Elevator::feedback[1].in_action,
                        Elevator::feedback[2].in_action,
                        Elevator::feedback[3].in_action);
                break;
            }

            Elevator::send_elevator_currents();
            sleep(TIME_MS2I(elevator_check_interval_));
        }
        chassis_target_vy_ = -1600;
        sleep(TIME_MS2I(700));
        chassis_target_vy_ = -500;
        sleep(TIME_MS2I(400));

        chassis_target_vy_ = 0;

        /** Complete **/
        LOG("[ELE UP] Complete");
        status_ = STOP;
        exit(0);
    } else if(status_== DOWNWARD){

    }
    // TODO: write action of going downward
}

