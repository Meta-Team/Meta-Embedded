//
// Created by liuzikai on 2019-06-25.
//

#include "inspector_infantry.h"

AbstractAHRS *InspectorI::ahrs = nullptr;
CANInterface *InspectorI::can1 = nullptr;
CANInterface *InspectorI::can2 = nullptr;

bool InspectorI::gimbal_failure_ = false;
bool InspectorI::chassis_failure_ = false;
bool InspectorI::remote_failure_ = false;

InspectorI::InspectorThread InspectorI::inspectorThread;

void InspectorI::init(CANInterface *can1_, CANInterface *can2_, AbstractAHRS *ahrs_) {
    can1 = can1_;
    can2 = can2_;
    ahrs = ahrs_;
}

void InspectorI::start_inspection(tprio_t thread_prio) {
    inspectorThread.start(thread_prio);
}

void InspectorI::startup_check_can() {
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

void InspectorI::startup_check_mpu() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        if (not WITHIN_RECENT_TIME(ahrs->get_mpu_update_time(), 5)) {
            // No signal in last 5 ms (normal interval 1 ms for on-board MPU)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void InspectorI::startup_check_ist() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        if (not WITHIN_RECENT_TIME(ahrs->get_ist_update_time(), 5)) {
            // No signal in last 5 ms (normal interval 1 ms for on-board MPU)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void InspectorI::startup_check_remote() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 50)) {
        if (not WITHIN_RECENT_TIME(Remote::last_update_time, 25)) {  // No signal in last 25 ms (normal interval 7 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(15);
    }
}

void InspectorI::startup_check_chassis_feedback() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        if (not WITHIN_RECENT_TIME(ChassisIF::feedback[ChassisIF::FR]->last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis FR offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(ChassisIF::feedback[ChassisIF::FL]->last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis FL offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(ChassisIF::feedback[ChassisIF::BL]->last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis BL offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(ChassisIF::feedback[ChassisIF::BR]->last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis BR offline.");
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

void InspectorI::startup_check_gimbal_feedback() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        if (not WITHIN_RECENT_TIME(GimbalIF::feedback[GimbalIF::FW_LEFT]->last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Gimbal FW_LEFT offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(GimbalIF::feedback[GimbalIF::FW_RIGHT]->last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Gimbal FW_RIGHT offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(GimbalIF::feedback[GimbalIF::YAW]->last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Gimbal Yaw offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(GimbalIF::feedback[GimbalIF::PITCH]->last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Gimbal Pitch offline.");
            t = SYSTIME;  // reset the counter
        }
        if (not WITHIN_RECENT_TIME(GimbalIF::feedback[GimbalIF::BULLET]->last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Gimbal Bullet offline.");
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

bool InspectorI::gimbal_failure() {
    return gimbal_failure_;
}

bool InspectorI::chassis_failure() {
    return chassis_failure_;
}

bool InspectorI::remote_failure() {
    return remote_failure_;
}

bool InspectorI::check_gimbal_failure() {
    bool ret = false;
    for (unsigned i = 0; i < 6; i++) {
        if (not WITHIN_RECENT_TIME(GimbalIF::feedback[i]->last_update_time, 20) &&
                                            GimbalIF::feedback[i]->type != CANInterface::NONE_MOTOR) {
            if (!gimbal_failure_) {  // avoid repeating printing
                LOG_ERR("Gimbal motor %u offline (at %u)", i, GimbalIF::feedback[i]->last_update_time);
            }
            ret = true;
        }
    }
    return ret;
}

bool InspectorI::check_chassis_failure() {
    bool ret = false;
    for (unsigned i = 0; i < ChassisIF::MOTOR_COUNT; i++) {
        // Check feedback, if the feedback's type is NONE_MOTOR, it's not valid
        if (not WITHIN_RECENT_TIME(ChassisIF::feedback[i]->last_update_time, 20) &&
                                            ChassisIF::feedback[i]->type != CANInterface::NONE_MOTOR) {
            if (!chassis_failure_) {  // avoid repeating printing
                LOG_ERR("Chassis motor %u offline (at %u)", i, ChassisIF::feedback[i]->last_update_time);
            }
            ret = true;
        }
    }
    return ret;
}

bool InspectorI::check_remote_data_error() {
    bool ret = (!ABS_IN_RANGE(Remote::rc.ch0, 1.1) || !ABS_IN_RANGE(Remote::rc.ch1, 1.1) ||
                !ABS_IN_RANGE(Remote::rc.ch2, 1.1) || !ABS_IN_RANGE(Remote::rc.ch3, 1.1) ||
                !(Remote::rc.s1 >= 1 && Remote::rc.s1 <= 3) || !(Remote::rc.s2 >= 1 && Remote::rc.s2 <= 3) ||
                !ABS_IN_RANGE(Remote::mouse.x, 1.1) || !ABS_IN_RANGE(Remote::mouse.y, 1.1) ||
                !ABS_IN_RANGE(Remote::mouse.z, 1.1) ||
                Remote::rx_buf_[12] > 1 || Remote::rx_buf_[13] > 1);
    return ret;

}

void InspectorI::InspectorThread::main() {
    setName("InspectorI");
    while (!shouldTerminate()) {

        if (check_remote_data_error()) {
            remote_failure_ = true;  // Set it to true to avoid problem when thread switches to User in the middle
            while (check_remote_data_error()) {
                Remote::uart_synchronize();
                sleep(TIME_MS2I(10));  // wait for another normal frame
            }
            remote_failure_ = false;
        }

        remote_failure_ = (not WITHIN_RECENT_TIME(Remote::last_update_time, 50));
        if (remote_failure_) LED::led_off(DEV_BOARD_LED_REMOTE);
        else LED::led_on(DEV_BOARD_LED_REMOTE);

        gimbal_failure_ = check_gimbal_failure();
        if (gimbal_failure_) LED::led_off(DEV_BOARD_LED_GIMBAL);
        else LED::led_on(DEV_BOARD_LED_GIMBAL);

        chassis_failure_ = check_chassis_failure();
        if (chassis_failure_) LED::led_off(DEV_BOARD_LED_CHASSIS);
        else LED::led_on(DEV_BOARD_LED_CHASSIS);

        if (remote_failure_ || gimbal_failure_ || chassis_failure_) {
            if (!BuzzerSKD::alerting()) BuzzerSKD::alert_on();
        } else {
            if (BuzzerSKD::alerting()) BuzzerSKD::alert_off();
        }

        sleep(TIME_MS2I(INSPECTOR_THREAD_INTERVAL));
    }
}