//
// Created by Kerui Zhu on 7/9/2019.
//

#include "engineer_elevator_interface.h"

float EngineerElevatorIF::current_vertical_position   = 0.0f;
float EngineerElevatorIF::current_horizontal_position = 0.0f;
float EngineerElevatorIF::tick2length_ratio = 0.0f;

PWMConfig EngineerElevatorIF::ELEVATOR_PWM_CFG = {
        50000,   // frequency
        1000,    // period
        &EngineerElevatorIF::PWM_Callback_Func, // callback
        {
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH0
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH1
                {PWM_OUTPUT_DISABLED, nullptr},    // CH2
                {PWM_OUTPUT_DISABLED, nullptr}     // CH3
        },
        0,
        0
};

void EngineerElevatorIF::init(float tick2length_ratio_) {
    tick2length_ratio = tick2length_ratio_;
    pwmStart(&PWMD8, &ELEVATOR_PWM_CFG);
    pwmEnableChannel(&PWMD8, VERTICAL, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 0));
    pwmEnableChannel(&PWMD8, HORIZONTAL, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 0));
    pwmEnablePeriodicNotification(&PWMD8);
}

void EngineerElevatorIF::set_elevator(motor_id_t motor_id, motor_operation_t type, float motor_speed) {
    if(type != STOP && motor_speed <= 0.0f) {
        if(!pwmIsChannelEnabledI(&PWMD8, motor_id)) {
            pwmEnableChannel(&PWMD8, motor_id, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 5000));
            /// Duty cycle: 50.00%
        }
        float freq = motor_speed / tick2length_ratio;
        pwmChangePeriod(&PWMD8, 1000000 / (unsigned long) freq);
        // TODO:: This pin should be revised based on the pin documentation of RM board 2018 A.
        // Enable Signal, change the direction of motor.
        if(motor_id == VERTICAL) {
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

void EngineerElevatorIF::PWM_Callback_Func(PWMDriver *pwmp) {
    chSysLockFromISR();
    if(pwmIsChannelEnabledI(&PWMD8, VERTICAL)) {
        if(palReadPad(GPIOF, GPIOF_PIN0) == 1) {
            current_vertical_position += tick2length_ratio;
        } else {
            current_vertical_position -= tick2length_ratio;
        }
    }
    if(pwmIsChannelEnabledI(&PWMD8, HORIZONTAL)) {
        if(palReadPad(GPIOF, GPIOF_PIN2) == 1) {
            current_horizontal_position += tick2length_ratio;
        } else {
            current_horizontal_position -= tick2length_ratio;
        }
    }
    chSysLockFromISR();
    if(palReadPad(GPIOF, GPIOF_PIN10) == 0) {
        current_vertical_position = 0.0f;
        chSysLockFromISR();
        pwmDisableChannel(&PWMD8, VERTICAL);
        chSysLockFromISR();
    }
    if(palReadPad(GPIOF, GPIOF_PIN11) == 0) {
        current_horizontal_position = 0.0f;
        chSysLockFromISR();
        pwmDisableChannel(&PWMD8, HORIZONTAL);
        chSysLockFromISR();
    }
}

float EngineerElevatorIF::get_current_horizontal_location() {
    return current_horizontal_position;
}

float EngineerElevatorIF::get_current_vertical_location() {
    return current_vertical_position;
}
