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

    static void init(float tick2length_ratio_);

    enum motor_id_t {
        VERTICAL, // right motor, 0
        HORIZONTAL, // left motor, 1
        MOTOR_COUNT
    };

    enum motor_operation_t {
        FORWARD,
        BACKWARD,
        STOP
    };

    static void set_elevator(motor_id_t motor_id, motor_operation_t type, float motor_speed);

    static float get_current_vertical_location();

    static float get_current_horizontal_location();

private:

    static PWMConfig ELEVATOR_PWM_CFG;

    static float current_vertical_position, current_horizontal_position, tick2length_ratio;

    static void PWM_Callback_Func(PWMDriver *pwmp);

};

#endif //META_INFANTRY_ENGINEER_ELEVATOR_INTERFACE_H
