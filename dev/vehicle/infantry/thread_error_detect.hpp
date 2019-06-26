//
// Created by liuzikai on 2019-05-11.
//

#ifndef META_INFANTRY_THREAD_ERROR_DETECT_HPP
#define META_INFANTRY_THREAD_ERROR_DETECT_HPP



inline void startupCheckRemote() {

}

inline void startupCheckGimbalFeedback() {

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
