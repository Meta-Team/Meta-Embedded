//
// Created by Kerui Zhu on 7/9/2019.
//

#include "engineer_elevator_interface.h"

const PWMConfig FRICTION_WHEELS_PWM_CFG = {
        50000,   // frequency
        1000,    // period
        nullptr, // callback
        {
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH0
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH1
                {PWM_OUTPUT_DISABLED, nullptr},    // CH2
                {PWM_OUTPUT_DISABLED, nullptr}     // CH3
        },
        0,
        0
};

void EngineerElevatorIF::init() {
    pwmStart(&PWMD8, &FRICTION_WHEELS_PWM_CFG);
    pwmEnableChannel(&PWMD8, LIFT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 0 * 500 + 500));
    pwmEnableChannel(&PWMD8, PUSH, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 0 * 500 + 500));
}

void EngineerElevatorIF::set_elevator(motor_id_t motor_id,  motor_operation_t type) {
    if(type != STOP) {
        pwmEnableChannel(&PWMD8, motor_id, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 0.5* 500 + 500));
        // TODO:: This pin should be revised based on the pin documentation of RM board 2018 A.
        // Enable Signal, change the direction of motor.
        if(motor_id == LIFT) {
            palWritePad(GPIOF, GPIOF_PIN0, type);
            palWritePad(GPIOF, GPIOF_PIN1, 1 - type);
        } else {
            palWritePad(GPIOF, GPIOF_PIN2, type);
            palWritePad(GPIOF, GPIOF_PIN3, 1 - type);
        }
    } else {
        pwmDisableChannel(&PWMD8, motor_id);
    }
}
