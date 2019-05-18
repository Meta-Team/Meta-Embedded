//
// Created by liuzikai on 2019-05-18.
//

#ifndef META_INFANTRY_THREAD_ACTION_TRIGGER_HPP
#define META_INFANTRY_THREAD_ACTION_TRIGGER_HPP

class ActionTriggerThread : public chibios_rt::BaseStaticThread<2048> {

    static constexpr int action_trigger_thread_interval = 20; // [ms]

    bool keyPressed = false;

    void main() final {

        setName("action_trigger");

        Elevator::change_pid_params({ELEVATOR_PID_A2V_PARAMS}, {ELEVATOR_PID_V2I_PARAMS});

        enum box_door_status_t {
            HIGH,
            LOW
        };

        box_door_status_t box_door_status = HIGH;
        palSetPad(GPIOH, GPIOH_POWER3_CTRL);

        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::S_DOWN) { // PC Mode

                if (Remote::key.v) {                                               // elevator lift up
                    if (!keyPressed) {
                        LOG_USER("press V");
                        Elevator::calc_back(-Elevator::STAGE_HEIGHT);
                        Elevator::calc_front(-Elevator::STAGE_HEIGHT);
                        keyPressed = true;
                    }
                } else if (Remote::key.b) {                                        // elevator lift down
                    if (!keyPressed) {
                        LOG_USER("press B");
                        Elevator::calc_back(0);
                        Elevator::calc_front(0);
                        keyPressed = true;
                    }
                } else if (Remote::key.z) {                                        // robotic arm initial outward
                    if (!keyPressed) {
                        LOG_USER("press Z");
                        keyPressed = true;
                    }
                    if (roboticArmThread.get_status() == roboticArmThread.STOP) {
                        if (!roboticArmThread.is_outward()) {
                            LOG("Trigger RA out");
                            roboticArmThread.start_initial_outward(NORMALPRIO - 3);
                        }
                    }
                } else if (Remote::key.x) {                                        // robotic arm fetch once
                    if (!keyPressed) {
                        LOG_USER("press X");
                        keyPressed = true;
                    }
                    if (roboticArmThread.get_status() == roboticArmThread.STOP) {
                        if (roboticArmThread.is_outward()) {
                            LOG("Trigger RA fetch");
                            roboticArmThread.start_one_fetch(NORMALPRIO - 3);
                        }
                    }
                } else if (Remote::key.c) {                                        // robotic arm final inward
                    if (!keyPressed) {
                        LOG_USER("press C");
                        keyPressed = true;
                    }
                    if (roboticArmThread.get_status() == roboticArmThread.STOP) {
                        if (roboticArmThread.is_outward()) {
                            LOG("Trigger RA in");
                            roboticArmThread.start_final_inward(NORMALPRIO - 3);
                        }
                    }
                } else if (Remote::key.g) {
                    if (!keyPressed) {
                        LOG_USER("press G");
                        keyPressed = true;
                    }
                    if (elevatorThread.get_status() == elevatorThread.STOP) {
                        elevatorThread.start_up_actions(NORMALPRIO - 2);
                    }
                } else if (Remote::key.f) {
                    if (!keyPressed) {
                        LOG_USER("press F");
                        keyPressed = true;
                        if (box_door_status == HIGH) {
                            LOG("Change to LOW");
                            box_door_status = LOW;
                            palClearPad(GPIOH, GPIOH_POWER3_CTRL);
                        }
                        else {
                            LOG("Change to HIGH");
                            box_door_status = HIGH;
                            palSetPad(GPIOH, GPIOH_POWER3_CTRL);
                        }
                    }
                }
                else {
                    keyPressed = false;
                }
            }

            sleep(TIME_MS2I(action_trigger_thread_interval));
        }
    }
}

#endif //META_INFANTRY_THREAD_ACTION_TRIGGER_HPP
