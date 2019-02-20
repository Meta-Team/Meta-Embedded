//
// Created by Zhu Kerui on 2019/1/16.
//

#ifndef META_INFANTRY_RMDS108_INTERFACE_H
#define META_INFANTRY_RMDS108_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

class ElevatorInterface {
public:

    enum wheel_t {
        FRONT_LEFT,
        FRONT_RIGHT,
        REAR_LEFT,
        REAR_RIGHT,
        WHEEL_COUNT // = 4
    };


    /**
     * @brief set the target position for both the front wheels and the rear wheels
     * @param front_wheel_position      the target position of the front wheels
     * @param rear_wheel_position       the target position of the rear wheels
     */
    static void set_target_position(int32_t front_wheel_position, int32_t rear_wheel_position);


    /** Motor Interface **/
    typedef struct {
        int32_t real_position; // the real position of the motor
        int16_t real_current;  // the real current in the motor
        int16_t real_velocity;  // the real velocity of the motor

    } elevator_wheel_t;

    /**
     * @brief contains the real information of the four wheels
     * elevator_wheels[0] and elevator_wheels[1] are for the front_left and front_right wheels accordingly
     * elevator_wheels[2] and elevator_wheels[3] are for the left_rear and right_rear wheels accordingly
     */
    static elevator_wheel_t elevator_wheels[4];

    /**
     * @brief Get the feedback (real position, real velocity, real current) of each motor from the driver
     * @return true if success, false otherwise
     */
    static bool process_feedback(CANRxFrame *rxmsg);


    /**
     * @brief send message of each motor
     * @return true if success, false otherwise
     */
    static bool send_target_position();

    /**
     * @brief set the CAN interface
     * @param can_interface
     */
    static void init(CANInterface* can_interface);

private:

    /**
     * @brief contains the target position of both the front wheels and the rear wheels
     * target_position[0] contains the target position of the front wheels
     * target_position[1] contains the target position of the rear wheels
     */
    static int32_t target_position[2];


    enum wheel_can_id_t {
        FRONT_LEFT_CAN_ID = 1,
        FRONT_RIGHT_CAN_ID = 2,
        REAR_LEFT_CAN_ID = 3,
        REAR_RIGHT_CAN_ID = 4
    };

    static CANInterface* can;

    static constexpr unsigned int can_group_id  = 3;
    static constexpr uint8_t feedback_interval = 100;
    static constexpr uint16_t driver_pwm = 2500;  // the pwm of the current

};

#endif //META_INFANTRY_RMDS108_INTERFACE_H
