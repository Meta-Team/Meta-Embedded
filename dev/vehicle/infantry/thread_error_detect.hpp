//
// Created by liuzikai on 2019-05-11.
//

#ifndef META_INFANTRY_THREAD_ERROR_DETECT_HPP
#define META_INFANTRY_THREAD_ERROR_DETECT_HPP

inline void startupCheckCAN() {
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 100) {
        if (StateHandler::fetchCANErrorMark()) {  // can error occurs
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(5);
    }
}

inline void startupCheckMPU6500() {
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 20) {
        if (SYSTIME - MPU6500::last_update_time > 3) {
            // No signal in last 3 ms (normal interval 1 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(3);
    }
}

inline void startupCheckRemote() {
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 50) {
        if (SYSTIME - Remote::last_update_time > 15) {
            // No signal in last 15 ms (normal interval 7 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(15);
    }
}

inline void startupCheckGimbalFeedback() {
    // TODO: echo to user which motor lose connection
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 50) {
        if (SYSTIME - Gimbal::feedback[Gimbal::YAW].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - Gimbal::feedback[Gimbal::PITCH].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - Gimbal::feedback[Gimbal::BULLET].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(3);
    }
}

inline void startupCheckChassisFeedback() {
    // TODO: echo to user which motor lose connection
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 50) {
        if (SYSTIME - Chassis::feedback[Chassis::FR].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - Chassis::feedback[Chassis::FL].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - Chassis::feedback[Chassis::BL].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        if (SYSTIME - Chassis::feedback[Chassis::BR].last_update_time > 3) {
            // No feedback in last 3 ms (normal 1 ms)
            t = SYSTIME;  // reset the counter
        }
        chThdSleepMilliseconds(3);
    }
}

/**
 * @name ErrorDetectThread
 * @brief Thread to detect error
 * @pre Startup self-check has pass
 */
class ErrorDetectThread : public chibios_rt::BaseStaticThread<1024> {

    static constexpr unsigned ERROR_DETECT_THREAD_INTERVAL = 50; // [ms]

    void main() final {

        setName("detect");

        while (!shouldTerminate()) {

            if (SYSTIME - MPU6500::last_update_time > 10) {
                StateHandler::raiseException(StateHandler::MPU6500_DISCONNECTED);
            }

            if (SYSTIME - Remote::last_update_time > 30) {
                StateHandler::raiseException(StateHandler::REMOTE_DISCONNECTED);
            }

            for (unsigned i = 0; i < Gimbal::MOTOR_COUNT; i++) {
                if (SYSTIME - Gimbal::feedback[i].last_update_time > 5) {
                    StateHandler::raiseException(StateHandler::GIMBAL_DISCONNECTED, i);
                }
            }

            for (unsigned i = 0; i < Chassis::MOTOR_COUNT; i++) {
                if (SYSTIME - Chassis::feedback[i].last_update_time > 5) {
                    StateHandler::raiseException(StateHandler::CHASSIS_DISCONNECTED, i);
                }
            }

            sleep(TIME_MS2I(ERROR_DETECT_THREAD_INTERVAL));
        }

    }

};


#endif //META_INFANTRY_THREAD_ERROR_DETECT_HPP
