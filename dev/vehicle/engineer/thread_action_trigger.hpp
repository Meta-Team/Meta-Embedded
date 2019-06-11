//
// Created by liuzikai on 2019-05-18.
//

#ifndef META_INFANTRY_THREAD_ACTION_TRIGGER_HPP
#define META_INFANTRY_THREAD_ACTION_TRIGGER_HPP

#include "thread_elevator.h"
#include "state_machine_bullet_fetch.h"
#include "state_machine_stage_climb.h"

class ActionTriggerThread : public chibios_rt::BaseStaticThread<2048> {

public:

    ActionTriggerThread(ElevatorThread &elevatorThread_,
                        BulletFetchStateMachine &bulletFetchStateMachine_,
                        StageClimbStateMachine &stageClimbStateMachine_)
            : elevatorThread(elevatorThread_),
              bulletFetchStateMachine(bulletFetchStateMachine_),
              stageClimbStateMachine(stageClimbStateMachine_) {};

private:

    ElevatorThread &elevatorThread;
    BulletFetchStateMachine &bulletFetchStateMachine;
    StageClimbStateMachine &stageClimbStateMachine;

    static constexpr int ACTION_TRIGGER_THREAD_INTERVAL = 20; // [ms]

    bool keyPressed = false;

    void main() final {

        setName("action");

        enum binary_status_t {
            HIGH,
            LOW
        };

        binary_status_t box_door_status = LOW;
//        binary_status_t drawer_status = LOW;
        palClearPad(GPIOH, GPIOH_POWER3_CTRL);
        palClearPad(GPIOH, GPIOH_POWER4_CTRL);
        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::S_DOWN) { // PC Mode

                if (Remote::key.v) {                                               // elevator lift up
                    if (!keyPressed) {
                        LOG_USER("click V");
                        elevatorThread.set_front_target_height(20);
                        elevatorThread.set_back_target_height(20);
                        keyPressed = true;
                    }
                } else if (Remote::key.b) {                                        // elevator lift down
                    if (!keyPressed) {
                        LOG_USER("click B");
                        elevatorThread.set_front_target_height(0);
                        elevatorThread.set_back_target_height(0);
                        keyPressed = true;}
                    keyPressed = true;
                    }
                } else if (Remote::key.z) {// robotic arm initial outward
                LOG_USER("GETHERE!");
                    if (!keyPressed) {
                        LOG_USER("click Z");
                        if (bulletFetchStateMachine.get_current_action() == bulletFetchStateMachine.STOP) {
                            LOG("Trigger RA out");
                            bulletFetchStateMachine.start_initial_outward(NORMALPRIO - 3);
                        } else {
                            LOG_WARN("RA in action");

                    }
                } else if (Remote::key.x) {                                        // robotic arm fetch once
                    if (!keyPressed) {
                        LOG_USER("click X");
                        if (bulletFetchStateMachine.get_current_action() == bulletFetchStateMachine.STOP) {
                            if (bulletFetchStateMachine.is_outward()) {
                                LOG("Trigger RA fetch");
                                bulletFetchStateMachine.start_one_fetch(NORMALPRIO - 3);
                            } else {
                                LOG_WARN("RA in not outward");
                            }
                        } else {
                            LOG_WARN("RA in action");
                        }
                        keyPressed = true;
                    }

                } else if (Remote::key.c) {                                        // robotic arm final inward
                    if (!keyPressed) {
                        LOG_USER("click C");
                        if (bulletFetchStateMachine.get_current_action() == bulletFetchStateMachine.STOP) {
                            if (bulletFetchStateMachine.is_outward()) {
                                LOG("Trigger RA in");
                                bulletFetchStateMachine.start_final_inward(NORMALPRIO - 3);
                            } else {
                                LOG_WARN("RA in not outward");
                            }
                        } else {
                            LOG_WARN("RA in action");
                        }
                        keyPressed = true;
                    }
                } else if (Remote::key.g) {                                        // climb up the stage
                    if (!keyPressed) {
                        LOG_USER("click G");
                        if (stageClimbStateMachine.get_current_action() == stageClimbStateMachine.STOP) {
                            LOG("Trigger SC up");
                            stageClimbStateMachine.start_up_action(NORMALPRIO - 2);
                        } else {
                            LOG_WARN("SC in action");
                        }
                        keyPressed = true;
                    }

                } else if (Remote::key.f) {                                        // switch the bullet box
                    if (!keyPressed) {
                        LOG_USER("click F");
                        keyPressed = true;
                        if (box_door_status == HIGH) {
                            LOG("BD change to LOW");
                            box_door_status = LOW;
                            palClearPad(GPIOH, GPIOH_POWER3_CTRL);
                        } else {
                            LOG("BD change to HIGH");
                            box_door_status = HIGH;
                            palSetPad(GPIOH, GPIOH_POWER3_CTRL);
                        }
                    }
                }
                else {
                    keyPressed = false;
                }
            }

            sleep(TIME_MS2I(ACTION_TRIGGER_THREAD_INTERVAL));
        }
    }
};

#endif //META_INFANTRY_THREAD_ACTION_TRIGGER_HPP
