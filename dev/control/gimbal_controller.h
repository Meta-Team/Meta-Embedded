//
// Created by liuzikai on 2019-01-05.
//

#ifndef META_INFANTRY_GIMBAL_CONTROLLER_H
#define META_INFANTRY_GIMBAL_CONTROLLER_H

#define GIMBAL_CONTROLLER_ENABLE_MIDDLE_VALUES 1

#include <vector>
#include "pid_controller.h"

/**
 * Module to perform calculation for gimbal controller
 */
class GimbalController {

public:

    typedef enum {
        YAW_ID = 0,
        PIT_ID = 1,
        BULLET_LOADER_ID =2
    } motor_id_t;

    typedef enum {
        MODE_1 = 0,
        MODE_2 = 1,
        MODE_3 = 2
    }shoot_mode_t;

    static std::vector<float> shoot_trigger_duty_cycle;  // the vector contains the duty cycles for different shoot modes

    class FrictionWheelController{

    };

    static FrictionWheelController frictionWheelController;

    /**
     * Controller for each motor
     */
    class MotorController{

    public:
        motor_id_t motor_id;   // motor id

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

        explicit MotorController(motor_id_t id) : motor_id(id){}
    };

    static MotorController yaw;
    static MotorController pitch;

    class BulletLoaderController{

    public:

        motor_id_t motor_id;

        int get_current(float measured_angle, float measured_velocity, float target_angle);

        PIDController angle_to_v_pid;
        PIDController v_to_i_pid;

        explicit BulletLoaderController(motor_id_t id) : motor_id(id){}

        bool shooting_accomplished = true;
    };
    static BulletLoaderController bullet_loader;


};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H
