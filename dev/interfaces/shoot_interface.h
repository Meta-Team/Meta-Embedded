//
// Created by Zhu Kerui on 2019/1/12.
//

#ifndef META_INFANTRY_SHOOT_INTERFACE_H
#define META_INFANTRY_SHOOT_INTERFACE_H
#include <stdint.h>
#include "remote_interpreter.h"

/**
 * Some important acronyms in this program:
 * FW: friction wheel
 * BL: bullet loader
 */

// ...

#define FRICTION_WHEEL_PWM_TIM PWMD8  //...

// Duty cycle of the PWM of the friction wheel:

#define FW_FULL_SPEED_DUTY_CYCLE 1  // The duty cycle for the friction wheel when it's at its maximum speed

// Bullet loader motor parameters

#define BL_MOTOR_BASE_CURRENT 700  // The base current of the bullet loader motor

#define BL_MOTOR_SPEED_DUTY_CYCLE_TRIGGER 0.1f  // The bullet loader motor will not start loading until the friction wheels reach this speed

#define BL_MOTOR_CHANGE_DIRECTION_CYCLE_COUNT 4  // The times that are needed for the bullet loader motor to turn a round


class ShootInterface {

public:

    // Bullet loader motor part

    bool loadEnable;  // Enable to load bullet or not

    int16_t bl_current;  // The current of the bullet loader motor

    uint16_t bl_actual_angle;  // The actual angle of the bullet loader motor

    int16_t bl_actual_rpm;  // The actual round of the bullet loader motor?

    // Friction wheel motors part

    float fw_motor_duty_cycle;

    enum friction_wheel_channel_t{
        FW_LEFT = 0,  // The left friction wheel (THE DIRECTION OF THE FRICTION WHEEL IS TEMPORARY
        FW_RIGHT = 1  // The right friction wheel
    };

    void set_zero_current();

    void set_shoot_mode(float speed_ratio);

    void send_shoot_currents();

    void shoot_calc_init();

    PWMConfig friction_wheels_pwmcfg = {

            50000,

            1000,

            NULL,

            {

                    {PWM_OUTPUT_ACTIVE_HIGH, NULL},

                    {PWM_OUTPUT_ACTIVE_HIGH, NULL},

                    {PWM_OUTPUT_DISABLED, NULL},

                    {PWM_OUTPUT_DISABLED, NULL}

            },

            0,

            0

    };
};


#endif //META_INFANTRY_SHOOT_INTERFACE_H
