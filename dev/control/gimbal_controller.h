//
// Created by liuzikai on 2019-01-05.
//

#ifndef META_INFANTRY_GIMBAL_CONTROLLER_H
#define META_INFANTRY_GIMBAL_CONTROLLER_H

#define GIMBAL_CONTROLLER_ENABLE_MIDDLE_VALUES 1

#include "pid_controller.h"

class GimbalController {

public:

    typedef enum {
        YAW_ID = 0,
        PIT_ID = 1
    } motor_id_t;

    class MotorController  {

    public:

        motor_id_t motor_id;   // motor id

        PIDController angle_to_v_pid;
        PIDController v_to_i_pid;

        float inline angle_to_v(float measured_angle, float target_angle) {
            return angle_to_v_pid.calc(measured_angle, target_angle);
        }

        float inline v_to_i(float measured_velocity, float target_velocity) {
            return v_to_i_pid.calc(measured_velocity, target_velocity);
        }

        explicit MotorController(motor_id_t id) : motor_id(id) {}

    };

    static MotorController yaw;
    static MotorController pitch;

};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H
