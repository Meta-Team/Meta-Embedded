//
// Created by Kerui Zhu on 7/22/2019.
//

#include "engineer_gimbal.h"

float EngineerGimbalIF::target_angle[2];
int EngineerGimbalIF::base = 250;
int EngineerGimbalIF::interval = 1000;

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
    target_angle[YAW] = yaw_angle_;
    target_angle[PIT] = pitch_angle_;
    pwmEnableChannel(&PWMD8, YAW, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, target_angle[YAW] / MAX_ANGLE * interval + base));
    pwmEnableChannel(&PWMD8, PIT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, target_angle[PIT] / MAX_ANGLE * interval + base));

}

void EngineerGimbalIF::init() {
    pwmStart(&PWMD8, &FRICTION_WHEELS_PWM_CFG);
    target_angle[YAW] = MAX_ANGLE / 2;
    target_angle[PIT] = MAX_ANGLE / 2;
}

float EngineerGimbalIF::get_target_angle(EngineerGimbalIF::gimbal_id_t id) {
    return target_angle[id];
}
