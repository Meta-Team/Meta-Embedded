//
// Created by liuzikai on 2019-06-25.
//

#include "inspector_engineer.h"

CANInterface *InspectorE::can1 = nullptr;
CANInterface *InspectorE::can2 = nullptr;

bool InspectorE::chassis_failure_ = false;
bool InspectorE::elevator_failure_ = false;
bool InspectorE::robotic_arm_failure_ = false;
bool InspectorE::remote_failure_ = false;

InspectorE::InspectorThread InspectorE::inspectorThread;
InspectorE::RefereeInspectorThread InspectorE::refereeInspectorThread;

void InspectorE::init(CANInterface *can1_, CANInterface *can2_) {
    can1 = can1_;
    can2 = can2_;
}

void InspectorE::start_inspection(tprio_t thread_prio, tprio_t referee_inspector_prio) {
    inspectorThread.start(thread_prio);
    refereeInspectorThread.start(referee_inspector_prio);
}

void InspectorE::startup_check_can() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 100)) {
        if (WITHIN_RECENT_TIME(can1->last_error_time, 5)) {  // can error occurs
            t = SYSTIME;  // reset the counter
        }
        if (WITHIN_RECENT_TIME(can2->last_error_time, 5)) {  // can error occurs
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void InspectorE::startup_check_remote() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 50)) {
        if (not WITHIN_RECENT_TIME(Remote::last_update_time, 25)) {  // No signal in last 25 ms (normal interval 7 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(15);
    }
}

void InspectorE::startup_check_chassis_feedback() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        if (not WITHIN_RECENT_TIME(ChassisIF::feedback[ChassisIF::FR].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis FR offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(ChassisIF::feedback[ChassisIF::FL].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis FL offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(ChassisIF::feedback[ChassisIF::BL].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis BL offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(ChassisIF::feedback[ChassisIF::BR].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis BR offline.");
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void InspectorE::startup_check_elevator_feedback() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        if (not WITHIN_RECENT_TIME(EngineerElevatorIF::elevatorMotor[EngineerElevatorIF::R].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Elevator R offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(EngineerElevatorIF::elevatorMotor[EngineerElevatorIF::L].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Elevator L offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(EngineerElevatorIF::aidedMotor[EngineerElevatorIF::R].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Aided R offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(EngineerElevatorIF::aidedMotor[EngineerElevatorIF::L].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Aided L offline.");
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void InspectorE::startup_check_robotic_arm_feedback() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        if (not WITHIN_RECENT_TIME(RoboticArmIF::motor_last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Robotic Arm motor offline.");
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

bool InspectorE::chassis_failure() {
    return chassis_failure_;
}

bool InspectorE::elevator_failure() {
    return elevator_failure_;
}

bool InspectorE::robotic_arm_failure() {
    return robotic_arm_failure_;
}

bool InspectorE::remote_failure() {
    return remote_failure_;
}

bool InspectorE::check_chassis_failure() {
    bool ret = false;
    for (unsigned i = 0; i < ChassisIF::MOTOR_COUNT; i++) {
        if (not WITHIN_RECENT_TIME(ChassisIF::feedback[i].last_update_time, 20)) {
            if (!chassis_failure_) {  // avoid repeating printing
                LOG_ERR("Chassis motor %u offline", i);
            }
            ret = true;
        }
    }
    return ret;
}

bool InspectorE::check_elevator_failure() {
    bool ret = false;
    if (not WITHIN_RECENT_TIME(EngineerElevatorIF::elevatorMotor[EngineerElevatorIF::R].last_update_time, 20)) {
        if (!elevator_failure_) {  // avoid repeating printing
            LOG_ERR("Elevator R offline.");
        }
        ret = true;
    }
    if (not WITHIN_RECENT_TIME(EngineerElevatorIF::elevatorMotor[EngineerElevatorIF::L].last_update_time, 20)) {
        if (!elevator_failure_) {  // avoid repeating printing
            LOG_ERR("Elevator L offline.");
        }
        ret = true;
    }
    if (not WITHIN_RECENT_TIME(EngineerElevatorIF::aidedMotor[EngineerElevatorIF::R].last_update_time, 20)) {
        if (!elevator_failure_) {  // avoid repeating printing
            LOG_ERR("Aided R offline.");
        }
        ret = true;
    }
    if (not WITHIN_RECENT_TIME(EngineerElevatorIF::aidedMotor[EngineerElevatorIF::L].last_update_time, 20)) {
        if (!elevator_failure_) {  // avoid repeating printing
            LOG_ERR("Aided L offline.");
        }
        ret = true;
    }
    return ret;
}

bool InspectorE::check_robotic_arm_failure() {
    bool ret = false;
    if (not WITHIN_RECENT_TIME(RoboticArmIF::motor_last_update_time, 20)) {
        if (!robotic_arm_failure_) {  // avoid repeating printing
            LOG_ERR("Robotic Arm motor offline.");
        }
        ret = true;
    }
    return ret;
}


bool InspectorE::check_remote_data_error() {
    bool ret = (!ABS_IN_RANGE(Remote::rc.ch0, 1.1) || !ABS_IN_RANGE(Remote::rc.ch1, 1.1) ||
                !ABS_IN_RANGE(Remote::rc.ch2, 1.1) || !ABS_IN_RANGE(Remote::rc.ch3, 1.1) ||
                !(Remote::rc.s1 >= 1 && Remote::rc.s1 <= 3) || !(Remote::rc.s2 >= 1 && Remote::rc.s2 <= 3) ||
                !ABS_IN_RANGE(Remote::mouse.x, 1.1) || !ABS_IN_RANGE(Remote::mouse.y, 1.1) ||
                !ABS_IN_RANGE(Remote::mouse.z, 1.1) ||
                Remote::rx_buf_[12] > 1 || Remote::rx_buf_[13] > 1);
    return ret;

}

void InspectorE::InspectorThread::main() {
    setName("InspectorE");
    while (!shouldTerminate()) {

        if (check_remote_data_error()) {
            remote_failure_ = true;  // Set it to true to avoid problem when thread switches to User in the middle
            while (check_remote_data_error()) {
                Remote::uart_synchronize();
                sleep(TIME_MS2I(10));  // wait for another normal frame
            }
            remote_failure_ = false;
        }

        remote_failure_ = (not WITHIN_RECENT_TIME(Remote::last_update_time, 30));
        if (remote_failure_) LED::led_off(DEV_BOARD_LED_REMOTE);
        else LED::led_on(DEV_BOARD_LED_REMOTE);

        chassis_failure_ = check_chassis_failure();
        if (chassis_failure_) LED::led_off(DEV_BOARD_LED_CHASSIS);
        else LED::led_on(DEV_BOARD_LED_CHASSIS);

        elevator_failure_ = check_elevator_failure();
        if (elevator_failure_) LED::led_off(DEV_BOARD_LED_ELEVATOR);
        else LED::led_on(DEV_BOARD_LED_ELEVATOR);

        robotic_arm_failure_ = check_robotic_arm_failure();
        if (robotic_arm_failure_) LED::led_off(DEV_BOARD_LED_ROBOTIC_ARM);
        else LED::led_on(DEV_BOARD_LED_ROBOTIC_ARM);

        if (remote_failure_ || chassis_failure_ || elevator_failure_ || robotic_arm_failure_) {
            if (!BuzzerSKD::alerting()) BuzzerSKD::alert_on();
        } else {
            if (BuzzerSKD::alerting()) BuzzerSKD::alert_off();
        }

        sleep(TIME_MS2I(INSPECTOR_THREAD_INTERVAL));
    }
}

void InspectorE::RefereeInspectorThread::main() {
    setName("InspectorH_Referee");

    chEvtRegisterMask(&Referee::data_received_event, &data_received_listener, DATA_RECEIVED_EVENTMASK);

    while (!shouldTerminate()) {

        chEvtWaitAny(DATA_RECEIVED_EVENTMASK);

        eventflags_t flags = chEvtGetAndClearFlags(&data_received_listener);
        (void) flags;

        // Toggle Referee LED if any data is received
        LED::led_toggle(DEV_BOARD_LED_REFEREE);
    }
}