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
        BULLET_LOADER_ID = 2
    } motor_id_t;

    typedef enum {
        STOP = 0,
        SLOW = 1,
        MIDDLE = 2,
        FAST = 3
    } shoot_mode_t;

    static float shoot_duty_cycles[4];  // the array contains the duty cycles for different shoot modes

    /**
     * Controller for each motor
     */
    class MotorController {

    public:
        motor_id_t motor_id;   // motor id

        PIDController angle_to_v_pid;
        PIDController v_to_i_pid;

        /**
         * @brief perform calculation with angle_to_v_pid
         * @param measured_angle [degree]
         * @param target_angle [degree]
         * @return target velocity [degree/s]
         */
        float angle_to_v(float measured_angle, float target_angle);

        /**
         * @brief perform calculation with v_to_i_pid
         * @param measured_velocity [degree/s]
         * @param target_velocity [degree/s]
         * @return target current [mA]
         */
        float v_to_i(float measured_velocity, float target_velocity);

        explicit MotorController(motor_id_t id) : motor_id(id) {}
    };

    class BulletLoaderController {
    public:

        motor_id_t motor_id;   // motor id

        PIDController v_to_i_pid;

        explicit BulletLoaderController(motor_id_t id) : motor_id(id) {}

        void start_continuous_shooting();

        void start_incontinuous_shooting(int bullet_num);

        void stop_shooting();

        float get_target_current(float measured_velocity, float target_velocity);

        void update_accumulation_angle(float accumulate_angle);

        void update_bullet(int bullet_changed = 0);

        /**
         * @brief get the number of the remained bullets
         * @return
         */
        int get_remained_bullet();

        bool get_shooting_status();

    private:

        float last_accumulate_angle = 0;

        bool continuous_shooting = false;
        bool shooting = false;
        float shoot_target_angle = 0; // in incontinuous mode, bullet loader stop if shoot_target_angle has been achieved
        float shoot_accumulate_angle = 0; // angle that is achieved during a single shoot

    private:

        /** Configurations **/
        float const one_bullet_step = 40.0; // degree
    };

    static MotorController yaw;
    static MotorController pitch;
    static BulletLoaderController bullet_loader;

private:

    static int remained_bullet;

};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H
