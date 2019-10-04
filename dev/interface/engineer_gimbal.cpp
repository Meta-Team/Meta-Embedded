//
// Created by Kerui Zhu on 7/22/2019.
//

#include "engineer_gimbal.h"

float EngineerGimbalIF::target_angle[2];

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
    VAL_CROP(yaw_angle_, MAX_ANGLE, 0);
    if (yaw_angle_ > 185 || yaw_angle_ < 25) VAL_CROP(pitch_angle_, 140.0, 60.0);
    else VAL_CROP(pitch_angle_, 95.0, 40.0);

    target_angle[YAW] = yaw_angle_;
    target_angle[PIT] = pitch_angle_;
    pwmEnableChannel(&PWMD8, YAW, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, target_angle[YAW] / MAX_ANGLE * 1000 + 250));
    pwmEnableChannel(&PWMD8, PIT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, target_angle[PIT] / MAX_ANGLE * 1000 + 250));

}

void EngineerGimbalIF::init() {
    pwmStart(&PWMD8, &FRICTION_WHEELS_PWM_CFG);
    set_target_angle(108.0f, 62.0f);
}

float EngineerGimbalIF::get_target_angle(EngineerGimbalIF::gimbal_id_t id) {
    return target_angle[id];
}
