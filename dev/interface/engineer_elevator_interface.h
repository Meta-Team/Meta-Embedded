//
// Created by Kerui Zhu on 7/9/2019.
//

#ifndef META_INFANTRY_ENGINEER_ELEVATOR_INTERFACE_H
#define META_INFANTRY_ENGINEER_ELEVATOR_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"
#include "common_macro.h"
#include "../vehicle/engineer/vehicle_engineer.h"
#include "sd_card_interface.h"

/**
 * @name EngineerElevatorIF
 * @brief interface to process elevator motor feedback and send target current.
 * @pre hardware is properly set. CAN id of each motor should be the same as motor_id_t.
 * @usage 1. Call init(CANInterface *). The interface should be properly initialized.
 *        2. Control the data flow based on actual implementation
 * @note This module is designed to process feedback automatically, but not to send current automatically, to avoid
 *       unintended elevator movements.
 */

#define EngineerLIFT

class EngineerElevatorIF {

public:

    /**
     * @brief initiate the Elevator interface. Start the PWM channel.
     */
    static void init();

    enum motor_operation_t {
        UP,
        DOWN,
        STOP
    };

    /**
     * @brief Set the elevator operation type. The motor_speed is actually the frequency of PWM,
     *        which stands for a tick for stepper motor.
     * @param motor_speed The speed of motor. Given a positive value would let elevator rise.
     */
    static void set_elevator(float motor_speed);

private:

    static PWMConfig ELEVATOR_PWM_CFG;

};

#endif //META_INFANTRY_ENGINEER_ELEVATOR_INTERFACE_H
