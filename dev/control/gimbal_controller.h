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

    class FrictionWheelController{
    public:

        typedef enum {
            MODE_1 = 0,
            MODE_2 = 1,
            MODE_3 = 2
        }mode_index_t;

        PIDController v_to_dc_pid;

        /**
         * @brief calculate the duty cycle from the velocity
         * @param measured_velocity
         * @param modeIndex
         * @return
         */
        float inline v_to_dc(float measured_velocity, mode_index_t modeIndex) {
            return v_to_dc_pid.calc(measured_velocity, friction_wheel_speed_modes[modeIndex]);
        }

    private:

        std::vector<float> friction_wheel_speed_modes {10.0, 20.0, 30.0};
    };

    static FrictionWheelController frictionWheelController;

    /**
     * Controller for each motor
     */
    class MotorController{

    public:
        motor_id_t motor_id;   // motor id

        float (*get_current)(float measured_angle, float measured_velocity, float target_angle);

        explicit MotorController(motor_id_t id) : motor_id(id){
            switch (motor_id){
                case YAW_ID:
                    get_current = get_yaw_current;
                    break;
                case PIT_ID:
                    get_current = get_pitch_current;
                    break;
                case BULLET_LOADER_ID:
                    get_current = get_bullet_loader_current;
                    break;
                default:
                    get_current = nullptr;
            }
        }
    };

    static MotorController yaw;
    static MotorController pitch;
    static MotorController bullet_loader;

    static bool start();

    static bool change_angle_to_v_pid(motor_id_t id, float _kp, float _ki, float _kd, float _i_limit, float _out_limit);

    static bool change_v_to_i_pid(motor_id_t id, float _kp, float _ki, float _kd, float _i_limit, float _out_limit);

private:

    static PIDController yaw_angle_to_v_pid;

    static PIDController yaw_v_to_i_pid;

    static PIDController pitch_angle_to_v_pid;

    static PIDController pitch_v_to_i_pid;

    static PIDController bullet_loader_angle_to_v_pid;

    static PIDController bullet_loader_v_to_i_pid;

    /**
     * @brief get the current to be sent
     * @param measured_angle
     * @param measured_velocity
     * @param target_angle
     * @return the current to be sent
     */
    static float get_bullet_loader_current(float measured_angle, float measured_velocity, float target_angle);

    /**
         * @brief get the current to be sent
         * @param measured_angle
         * @param measured_velocity
         * @param target_angle
         * @return the current to be sent
         */
    static float get_yaw_current(float measured_angle, float measured_velocity, float target_angle);

    /**
     * @brief get the current to be sent
     * @param measured_angle
     * @param measured_velocity
     * @param target_angle
     * @return the current to be sent
     */
    static float get_pitch_current(float measured_angle, float measured_velocity, float target_angle);

};


#endif //META_INFANTRY_GIMBAL_CONTROLLER_H
