//
// Created by liuzikai on 2019-01-05.
//

#ifndef META_INFANTRY_GIMBAL_CONTROLLER_H
#define META_INFANTRY_GIMBAL_CONTROLLER_H

#define GIMBAL_CONTROLLER_ENABLE_MIDDLE_VALUES 1

#include "pid_controller.h"

/**
 * Module to perform calculation for gimbal controller
 */
class GimbalController {

public:

    typedef enum {
        YAW_ID = 0,
        PIT_ID = 1
    } motor_id_t;

    /**
     * Controller for each motor
     */
    class MotorController  {

    public:

        motor_id_t motor_id;   // motor id

        /**
         * Two PID controller
         * They are made public. Directly access them to change parameters.
         */
        PIDController angle_to_v_pid;
        PIDController v_to_i_pid;

        /**
         * @brief perform calculation with angle_to_v_pid
         * @param measured_angle
         * @param target_angle
         * @return target velocity
         */
        float inline angle_to_v(float measured_angle, float target_angle) {
            return angle_to_v_pid.calc(measured_angle, target_angle);
        }

        /**
         * @brief perform calculation with v_to_i_pid
         * @param measured_velocity
         * @param target_velocity
         * @return target current
         */
        float inline v_to_i(float measured_velocity, float target_velocity) {
            return v_to_i_pid.calc(measured_velocity, target_velocity);
        }

        /**
         * @brief default constructor
         * @param id
         * @note set PID parameters though access PID controller modules
         */
        explicit MotorController(motor_id_t id) : motor_id(id) {}

    };

    static MotorController yaw;
    static MotorController pitch;

};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H
