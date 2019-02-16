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
        PIT_ID = 1,
        BULLET_LOADER_ID =2
    } motor_id_t;

    typedef enum {
        MODE_1 = 0,
        MODE_2 = 1,
        MODE_3 = 2
    }shoot_mode_t;

    class FrictionWheelController{

        float actual_duty_cycle = 0.0;  // THIS IS A PSEUDO PARAMETER, IT SHOULD BE REPLACED WITH FEEDBACK FROM FRICTION WHEEL IF NECESSARY

        float trigger_duty_cycle = 0.2;  // Bullet loader only works when the friction wheel duty cycle is over the trigger

        friend GimbalController;
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
    static MotorController bullet_loader;

    static void update_bullet(int bullet_changed = 0);

    /**
     * @brief return the target duty cycle for the friction wheels
     * @return
     */
    static float get_fw_pid();

    /**
     * @brief get the number of the remained bullets
     * @return
     */
    static int get_remained_bullet();

private:

    static int remained_bullet;

    static float shoot_trigger_duty_cycle[3];  // the array contains the duty cycles for different shoot modes

};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H
