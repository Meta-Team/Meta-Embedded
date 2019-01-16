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

    typedef enum {
        FRONT_LEFT = 0x005,  // The front-left wheel (the id is temporary)
        FRONT_RIGHT = 0x015,  // The front-right wheel (the id is temporary)
        REAR_LEFT = 0x025,  // The left-rear wheel (the id is temporary)
        REAR_RIGHT = 0x035  // The right-rear wheel (the id is temporary)
    } elevator_wheel_id_t;

    bool enabled;  // if not enabled, 0 current will be sent in send_elevator_currents

    /**
     * @brief contains the target position of both the front wheels and the rear wheels
     * target_position[0] contains the target position of the front wheels
     * target_position[1] contains the target position of the rear wheels
     */
    int32_t target_position[2];

    /**
     * @brief set the target position for both the front wheels and the rear wheels
     * @param front_wheel_position      the target position of the front wheels
     * @param rear_wheel_position       the target position of the rear wheels
     */
    void set_position(int32_t front_wheel_position, int32_t rear_wheel_position);

    /**
     * @brief contains the command CANTxFrames for the four wheels
     * txFrames[0] and txFrames[1] are for the front_left and front_right wheels accordingly
     * txFrames[2] and txFrames[3] are for the left_rear and right_rear wheels accordingly
     */
    CANTxFrame txFrames[4];

    /** Motor Interface **/
    typedef struct {

        /**
         * Basic Parameters
         */
        int32_t real_position; // the real position of the motor
        int16_t real_current;  // the real current in the motor
        int16_t real_velocity;  // the real velocity of the motor

    } elevator_wheel_t;

    uint16_t PWM;  // the pwm of the current

    /**
     * @brief contains the real information of the four wheels
     * elevator_wheels[0] and elevator_wheels[1] are for the front_left and front_right wheels accordingly
     * elevator_wheels[2] and elevator_wheels[3] are for the left_rear and right_rear wheels accordingly
     */
    elevator_wheel_t elevator_wheels[4];

    /**
     * @brief Get the feedback (real position, real velocity, real current) of each motor from the driver
     * @return true if success, false otherwise
     */
    bool get_feedback(CANTxFrame *rxmsg);


    /**
     * @brief send message of each motor
     * @return true if success, false otherwise
     */
    bool send_message();

    /**
     * Default constructor
     */
    ElevatorInterface() {
        enabled = false;
        txFrames[0].SID = FRONT_LEFT;
        txFrames[1].SID = FRONT_RIGHT;
        txFrames[2].SID = REAR_LEFT;
        txFrames[3].SID = REAR_RIGHT;
    }

    /**
     * @brief set the CAN interface
     * @param can_interface
     */
    static void set_can_interface (CANInterface* can_interface) {
        can = can_interface;
    }

private:

    static CANInterface* can;

};

#endif //META_INFANTRY_RMDS108_INTERFACE_H
