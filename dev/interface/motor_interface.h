//
// Created by root on 2021/1/25.
//

/**
 * @file    motor_interface.h
 * @brief   Interface to interact with low level driver of gimbal, including processing chassis motor feedback and
 *          sending target currents.
 *
 * @addtogroup gimbal
 * @{
 */

#ifndef META_INFANTRY_MOTORIF_H
#define META_INFANTRY_MOTORIF_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

/// Board Guard
#if defined(BOARD_RM_2018_A)
#elif defined(BOARD_RM_2017)
#else
#error "MotorIF has not been defined for selected board"
#endif


class MotorIF {

public:

    static constexpr unsigned MAXIMUM_MOTOR_COUNT = 16;

    enum motor_can_channel_t {
        none_can_channel,
        can_channel_1,
        can_channel_2
    };

    struct motor_can_config_t {
        motor_can_channel_t motor_can_channel;
        unsigned motor_can_id;
        CANInterface::motor_type_t motor_type;
        unsigned maximum_current;
        unsigned set_front_angle_raw;
        unsigned  front_angle_raw;
    };

    /**
     * Initialize GimbalIF. Angles of bullet loader and bullet plate will be reset.
     * @param can1_interface          Initialized CANInterface for yaw, pitch and bullet_loader motor
     * @param can2_interface          Initialized CANInterface for yaw, pitch and bullet_loader motor
     * @param motor_can_config        A group that contains ***ALL*** motor's info, include (can_channel, motor_can_id, motor_type)
     */

    static void init( CANInterface *can1_interface,                      CANInterface *can2_interface,
                      motor_can_config_t motor_can_config[],             int MOTOR_COUNT);

    /**
     * Motor feedback structure
     */
    static CANInterface::motor_feedback_t *feedback[MAXIMUM_MOTOR_COUNT];

    /**
     * set_target_current
     */
     static void set_target_current(int current, int motor_id);

    /**
     * get_target_current
     */
    static int get_target_current(int motor_id);

private:

    static CANInterface *can1_;
    static CANInterface *can2_;

    /**
     * Target current array in the order defined in motor_id_t
     */
    static int *target_current[MAXIMUM_MOTOR_COUNT];

    /**
     * Maximum value for target current array in the order defined in motor_id_t
     */
    static uint16_t max_target_current[MAXIMUM_MOTOR_COUNT];

    static int motor_count;

    friend CANInterface;

//#if GIMBAL_INTERFACE_ENABLE_VELOCITY_DIFFERENTIAL
//    static constexpr int VELOCITY_SAMPLE_INTERVAL = 50;  // count of feedback for one sample of angular velocity
//#endif

};


#endif //META_INFANTRY_MOTORIF_H
