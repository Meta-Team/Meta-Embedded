//
// Created by liuzikai on 2019-01-05.
//

#ifndef META_INFANTRY_GIMBAL_CONTROLLER_H
#define META_INFANTRY_GIMBAL_CONTROLLER_H

#define GIMBAL_CONTROLLER_ENABLE_MIDDLE_VALUES 1

#include "gimbal_interface.h"
#include "pid_controller.hpp"

/**
 * @name Gimbal
 * @brief
 * @note
 */
class Gimbal : public GimbalInterface, public PIDControllerBase {

public:

    /**
     * Initialize the gimbal and the shooter
     * @param can_interface           initialized CANInterface for yaw, pitch and bullet_loader motor
     * @param yaw_front_angle_raw     raw angle of yaw when gimbal points straight forward, depending on installation.
     * @param pitch_front_angle_raw   raw angle of pitch when gimbal points straight forward, depending on installation.
     * @param yaw_a2v_params
     * @param yaw_v2i_params
     * @param pitch_a2v_params
     * @param pitch_v2i_params
     * @param degree_per_bullet       Rotation degree of bullet loader motor for each motor
     */
    static void init(CANInterface *can_interface, uint16_t yaw_front_angle_raw, uint16_t pitch_front_angle_raw,
                     pid_params_t yaw_a2v_params, pid_params_t yaw_v2i_params,
                     pid_params_t pitch_a2v_params, pid_params_t pitch_v2i_params,
                     float degree_per_bullet);

    /**
     * Perform calculation from angle to velocity and put result into target_velocity_[]
     * @param id
     * @param actual_angle
     * @param target_angle
     * @note for outside code, only for test.
     */
    static void calc_a2v_(motor_id_t id_, float actual_angle_, float target_angle_);

    /**
     * Perform calculation from velocity to current and put result into target_current[]
     * @param id
     * @param actual_velocity
     * @param target_velocity
     * @note for outside code, only for test.
     */
    static void calc_v2i_(motor_id_t id_, float actual_velocity_, float target_velocity_);

    /**
     * Perform calculation from angle to current and put result into target_current[]
     * @param yaw_actual_velocity
     * @param pitch_actual_velocity
     * @param yaw_target_angle
     * @param pitch_target_angle
     */
    static void calc_gimbal(float yaw_actual_velocity, float pitch_actual_velocity,
                            float yaw_target_angle, float pitch_target_angle);

    /**
     * Perform calculation from velocity to current and put result into target_current[]
     * @param bullet_per_second   shoot speed
     */
    static void calc_bullet_loader(float bullet_per_second);

    static void add_bullet(int num);

    static int get_remaining_bullet();

private:

    /**
     * @brief calculated target velocity
     * @note middle value. For outside code, only for test.
     */
    static float target_velocity[MOTOR_COUNT];

    static PIDController a2v_pid[MOTOR_COUNT];
    static PIDController v2i_pid[MOTOR_COUNT];


    static int remaining_bullet_;

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

    static BulletLoaderController bullet_loader;

private:

    static int remained_bullet;

};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H
