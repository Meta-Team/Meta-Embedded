//
// Created by Kerui Zhu on 7/9/2019.
//

#include "engineer_elevator_interface.h"

PWMConfig EngineerElevatorIF::ELEVATOR_PWM_CFG = {
        100000,   // frequency
        500,    // period
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
    pwmStart(&PWMD8, &ELEVATOR_PWM_CFG);
    //pwmEnableChannel(&PWMD8, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 500));
}

void EngineerElevatorIF::set_elevator(float motor_speed) {

    if(motor_speed == 0) {
        pwmDisableChannel(&PWMD8, 0);
        pwmDisableChannel(&PWMD8, 1);
    } else {
        if(!pwmIsChannelEnabledI(&PWMD8, 0)) {
            pwmEnableChannel(&PWMD8, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 500));
            /// Duty cycle: 50.00%
        }
        if(!pwmIsChannelEnabledI(&PWMD8, 1)) {
            pwmEnableChannel(&PWMD8, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 500));
        }
        pwmChangePeriod(&PWMD8, (int)(100000 / (unsigned long) ABS(motor_speed)));
        if (motor_speed > 0) {
            palWritePad(GPIOF, GPIOF_PIN0, PAL_LOW);
            palWritePad(GPIOF, GPIOF_PIN1, PAL_HIGH);
        } else {
            palWritePad(GPIOF, GPIOF_PIN0, PAL_HIGH);
            palWritePad(GPIOF, GPIOF_PIN1, PAL_LOW);
        }
    }
}
