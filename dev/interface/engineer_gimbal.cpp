//
// Created by Kerui Zhu on 7/22/2019.
//

#include "engineer_gimbal.h"

float EngineerGimbalIF::yaw_angle;
float EngineerGimbalIF::pitch_angle;

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

void EngineerGimbalIF::set_target_angle(float yaw_angle_, float pitch_angle_) {
    yaw_angle = yaw_angle_;
    pitch_angle = pitch_angle_;
}

void EngineerGimbalIF::send_current() {

    pwmEnableChannel(&PWMD8, YAW, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, yaw_angle / MAX_ANGLE * 500 + 500));
    pwmEnableChannel(&PWMD8, PIT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, pitch_angle / MAX_ANGLE * 500 + 500));
}

void EngineerGimbalIF::init() {
    pwmStart(&PWMD8, &FRICTION_WHEELS_PWM_CFG);
    yaw_angle = MAX_ANGLE / 2;
    pitch_angle = MAX_ANGLE / 2;
}
