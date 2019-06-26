//
// Created by 钱晨 on 2019-05-18.
//

#ifndef META_INFANTRY_THREAD_ERROR_DETECT_HPP
#define META_INFANTRY_THREAD_ERROR_DETECT_HPP

#include "state_handler.h"
#include "mpu6500.h"
#include "remote_interpreter.h"
#include "gimbal.h"
#include "chassis.h"
#include <math.h>

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
        chThdSleepMilliseconds(5);
    }
}

inline void startupCheckRemote() {
    time_msecs_t t = SYSTIME;
    while (SYSTIME - t < 50) {
        if (SYSTIME - Remote::last_update_time > 15) {
            // No signal in last 15 minutes
            t = SYSTIME;
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
        if (SYSTIME - Gimbal::feedback[Gimbal::PLATE].last_update_time > 3) {
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
        float loader_angle_sequence[3];
        float plate_angle_sequence[3];

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

            //if (runtime > 130) StateHandler::raiseException(StateHandler::BULLET_LOADER_STUCK);
            //if (runtime > 40 && Shoot::feedback[2].actual_current > 700) StateHandler::raiseException(StateHandler::BULLET_LOADER_STUCK);

            // update 3 frames of angle.
            loader_angle_sequence[0] = loader_angle_sequence[1];
            loader_angle_sequence[1] = loader_angle_sequence[2];
            loader_angle_sequence[2] = Shoot::feedback[2].actual_angle;

            plate_angle_sequence[0] = plate_angle_sequence[1];
            plate_angle_sequence[1] = plate_angle_sequence[2];
            plate_angle_sequence[2] = Shoot::feedback[3].actual_angle;

            // if the the target angle has sent for several times and the loader do not respond in 0.15 second, then the program regard the loader has stuck.
            if(!loader_stop[2] && (fabs(loader_angle_sequence[0] - loader_angle_sequence[1]) < 2.0f && fabs(loader_angle_sequence[1] - loader_angle_sequence[2]) < 2.0f)){
                StateHandler::raiseException(StateHandler::BULLET_LOADER_STUCK);
            }


            sleep(TIME_MS2I(ERROR_DETECT_THREAD_INTERVAL));
        }

    }

};


#endif //META_INFANTRY_THREAD_ERROR_DETECT_HPP
