//
// Created by liuzikai on 2019-06-25.
// Edited by Qian Chen & Mo Kanya on 2019-07-05
//

#include "inspector.h"

AbstractAHRS *Inspector::ahrs = nullptr;
CANInterface *Inspector::can1 = nullptr;
CANInterface *Inspector::can2 = nullptr;

bool Inspector::gimbal_failure_ = false;
bool Inspector::chassis_failure_ = false;
bool Inspector::remote_failure_ = false;

Inspector::InspectorThread Inspector::inspectorThread;

void Inspector::init(CANInterface *can1_, CANInterface *can2_, AbstractAHRS *ahrs_) {
    can1 = can1_;
    can2 = can2_;
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
        if (SYSTIME - ahrs->get_mpu_update_time() > 5) {  // No signal in last 5 ms (normal interval 1 ms for on-board MPU)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void Inspector::startup_check_ist() {
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 20) {
        if (SYSTIME - ahrs->get_ist_update_time() > 5) {  // No signal in last 5 ms (normal interval 1 ms for on-board MPU)
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
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 20) {
        if (SYSTIME - ChassisIF::feedback[ChassisIF::FR].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis FR offline.");
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - ChassisIF::feedback[ChassisIF::FL].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis FL offline.");
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - ChassisIF::feedback[ChassisIF::BL].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis BL offline.");
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - ChassisIF::feedback[ChassisIF::BR].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis BR offline.");
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void Inspector::startup_check_gimbal_feedback() {
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 20) {
        if (SYSTIME - GimbalIF::feedback[GimbalIF::YAW].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Gimbal Yaw offline.");
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - GimbalIF::feedback[GimbalIF::PITCH].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Gimbal Pitch offline.");
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - GimbalIF::feedback[GimbalIF::BULLET].last_update_time > 5) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Gimbal Bullet offline.");
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
    for (unsigned i = 0 ; i < 3; i++) {
        if (SYSTIME - GimbalIF::feedback[i].last_update_time > 20) {
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
                LOG_ERR("Chassis motor %u offline", i);
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
        if (remote_failure_) LED::led_off(4);
        else LED::led_on(4);

        gimbal_failure_ = check_gimbal_failure();
        if (gimbal_failure_) LED::led_off(5);
        else LED::led_on(5);

        chassis_failure_ = check_chassis_failure();
        if (chassis_failure_) LED::led_off(6);
        else LED::led_on(6);

        if (remote_failure_ || gimbal_failure_ || chassis_failure_) {
            if (!Buzzer::alerting())  Buzzer::alert_on();
        } else {
            if (Buzzer::alerting())  Buzzer::alert_off();
        }

        sleep(TIME_MS2I(INSPECTOR_THREAD_INTERVAL));
    }
}