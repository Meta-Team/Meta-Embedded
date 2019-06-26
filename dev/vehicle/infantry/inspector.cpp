//
// Created by liuzikai on 2019-06-25.
//

#include "inspector.h"

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