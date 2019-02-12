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

    };

    static FrictionWheelController frictionWheelController;

    /**
     * Controller for each motor
     */
    class MotorController{

    public:
        motor_id_t motor_id;   // motor id

        float angular_velocity = 0.0f;

        float actual_angle = 0.0f;

        float target_angle = 0.0f;

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

    static void start();

    static bool update_motor_data(motor_id_t motor_id, float actual_angle, float angular_velocity);

    static void shoot_bullet(shoot_mode_t shoot_mode, int bullet_num);

    static float get_fw_pid(shoot_mode_t shoot_mode);

    static int get_bullet_loader_target_current();

    /**
     * @brief Called when shooting or reloading happens
     * @param new_bullet_added
     * @return
     */
    static int update_bullet_count(int new_bullet_added = 0);

    /**
     * @brief get the number of the remained bullets
     * @return
     */
    static int get_remained_bullet();

private:

    static float one_bullet_step;

    static int remained_bullet;

    static float actual_duty_cycle;  // THIS IS A PSEUDO PARAMETER, IT SHOULD BE REPLACED WITH FEEDBACK FROM FRICTION WHEEL IF NECESSARY

    static float trigger_duty_cycle;  // Bullet loader only works when the friction wheel duty cycle is over the trigger

    static float shoot_trigger_duty_cycle[3];  // the array contains the duty cycles for different shoot modes
};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H
