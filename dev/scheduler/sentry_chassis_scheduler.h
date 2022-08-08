//
// Created by zhukerui on 2019/4/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H

#include "ch.hpp"
#include "hal.h"
#include "pid_controller.hpp"
#include "can_motor_controller.h"

class SChassisSKD {
public:

    enum mode_t {
        FORCED_RELAX_MODE,    ///< Stop immediately and then enter forced relax mode.
        VELOCITY_MODE,        ///< Velocity control mode, for manual operation.
        POSITION_MODE         ///< Absolute position on rail.
    };

    enum install_direction_t {
        POSITIVE = 1,         ///< Axis CCW
        NEGATIVE = -1         ///< Axis CW
    };

    /**
     * @brief Local motor id of sentry chassis, NOT THE IDs IN CAN MOTOR DRIVER CLASSES.
     */
    enum chassis_motor_id_t {
        MOTOR_LEFT,           ///< Left motor
        MOTOR_RIGHT           ///< Right motor
    };

    /**
     * @brief Start sentry chassis scheduler
     * @param left_motor_install  [in] The installation direction of left motor.
     * @param right_motor_install [in] The installation direction of right motor.
     * @param thread_prio         [in] Thread priority for sentry chassis thread.
     */
    static void start(install_direction_t left_motor_install, install_direction_t right_motor_install,
                      tprio_t thread_prio);

    /**
     * @brief Set mode of sentry chassis scheduler.
     * @param mode [in] Mode of sentry chassis. Available options: FORCED_RELAX_MODE,
     *        VELOCITY_MODE, POSITION_MODE.
     */
    static void set_mode(mode_t mode);

    /**
     * @brief Set current position as origin.
     */
    static void reset_origin();

    /**
     * @brief Set the target position, bigger position will drive the chassis towards right.
     *        Only available in position mode.
     * @param dist the target position in [mm].
     */
    static void set_destination(float dist);

    /**
     * @brief Set the velocity of chassis. Only available in velocity mode.
     * @param velocity
     */
    static void set_velocity(float velocity);

    /**
     * @brief Get sentry's position.
     * @return The position of sentry from origin, in [mm].
     */
    static float present_position();

    /**
     * @brief Get sentry's speed.
     * @return Sentry's speed in [mm/s]
     */
    static float present_velocity();

private:

    // Local storage
    static mode_t mode;
    static install_direction_t motor_install_[2];


    static float target_position;  // [mm], bigger position will drive the chassis towards right
    static float target_velocity;  // [mm/s], calculated target velocity, middle value

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]
        void main() final;
    };

    constexpr static float schassis_wheel_curriculum = 55;
    static SKDThread skdThread;
};


#endif //META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H