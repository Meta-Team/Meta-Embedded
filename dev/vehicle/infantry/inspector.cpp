//
// Created by liuzikai on 2019-06-25.
//

#include "inspector.h"

AbstractAHRS *Inspector::ahrs = nullptr;
CANInterface *Inspector::can1 = nullptr;

bool Inspector::gimbal_failure_ = false;
bool Inspector::chassis_failure_ = false;
bool Inspector::remote_failure_ = false;

Inspector::InspectorThread Inspector::inspectorThread;

void Inspector::init(CANInterface *can1_, AbstractAHRS *ahrs_) {
    can1 = can1_;
    ahrs = ahrs_;
}

void Inspector::start_inspection(tprio_t thread_prio) {
    inspectorThread.start(thread_prio);
}

void Inspector::startup_check_can() {
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 100) {
        if (SYSTIME - can1->last_error_time < 5) {  // can error occurs
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void Inspector::startup_check_mpu() {
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 20) {
        if (SYSTIME - ahrs->mpu_update_time > 5) {  // No signal in last 5 ms (normal interval 1 ms for on-board MPU)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void Inspector::startup_check_ist() {
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 20) {
        if (SYSTIME - ahrs->ist_update_time > 5) {  // No signal in last 5 ms (normal interval 1 ms for on-board MPU)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void Inspector::startup_check_remote() {
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 50) {
        if (SYSTIME - Remote::last_update_time > 25) {  // No signal in last 25 ms (normal interval 7 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(15);
    }
}

void Inspector::startup_check_chassis_feedback() {
    // TODO: echo to user which motor lose connection
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 20) {
        if (SYSTIME - ChassisIF::feedback[ChassisIF::FR].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - ChassisIF::feedback[ChassisIF::FL].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - ChassisIF::feedback[ChassisIF::BL].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - ChassisIF::feedback[ChassisIF::BR].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void Inspector::startup_check_gimbal_feedback() {
    // TODO: echo to user which motor lose connection
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 20) {
        if (SYSTIME - GimbalIF::feedback[GimbalIF::YAW].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - GimbalIF::feedback[GimbalIF::PITCH].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - GimbalIF::feedback[GimbalIF::BULLET].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

bool Inspector::gimbal_failure() {
    return gimbal_failure_;
}

bool Inspector::chassis_failure() {
    return chassis_failure_;
}

bool Inspector::remote_failure() {
    return remote_failure_;
}

bool Inspector::check_gimbal_failure() {
    bool ret = false;
    for (auto & i : GimbalIF::feedback) {
        if (SYSTIME - i.last_update_time > 20) {
            if (!gimbal_failure_) {  // avoid repeating printing
                LOG_ERR("Gimbal motor %u offline");
                ret = true;
            }
        }
    }
    return ret;
}

bool Inspector::check_chassis_failure() {
    bool ret = false;
    for (unsigned i = 0; i < ChassisIF::MOTOR_COUNT; i++) {
        if (SYSTIME - ChassisIF::feedback[i].last_update_time > 20) {
            if (!chassis_failure_) {  // avoid repeating printing
                LOG_ERR("Chassis motor %u offline");
                ret = true;
            }
        }
    }
    return ret;
}

void Inspector::InspectorThread::main() {
    setName("Inspector");
    while (!shouldTerminate()) {

        remote_failure_ = (SYSTIME - Remote::last_update_time > 30);
        gimbal_failure_ = check_gimbal_failure();
        chassis_failure_ = check_chassis_failure();

        sleep(TIME_MS2I(INSPECTOR_THREAD_INTERVAL));
    }
}