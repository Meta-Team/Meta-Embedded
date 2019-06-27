//
// Created by liuzikai on 2018/7/16.
//

/**
 * @file    led.cpp
 * @brief   Interface to operate on-board LEDs.
 * @note    This file is needed to avoid multiple definitions of set_led_when_halt()
 *
 * @addtogroup LED
 * @{
 */

#include "led.h"

extern "C" {
void set_led_when_halt(void) {

    /** Turn on red led to indicate halting */
    LED_PAD_ON(GPIOF, GPIOE_LED_RED);    // LED_RED_ON
    LED_PAD_OFF(GPIOF, GPIOF_LED_GREEN); // LED_GREEN_OFF

    /** Play a single sound to indicate halting (see Buzzer module) */
    PWMConfig pwm_config = {
            1000000,
            1000000, // Default note: 1Hz
            nullptr,
            {
                    {PWM_OUTPUT_ACTIVE_HIGH, nullptr},  // it's all CH1 for current support boards
                    {PWM_OUTPUT_DISABLED, nullptr},
                    {PWM_OUTPUT_DISABLED, nullptr},
                    {PWM_OUTPUT_DISABLED, nullptr}
            },
            0,
            0
    };

#if defined(BOARD_RM_2018_A)  // RM_BOARD_2018_A Buzzer: PH6 TIM12 CH1
#define BUZZER_PWM_DRIVER PWMD12
#elif defined(BOARD_RM_2017)  // RM_BOARD_2017 Buzzer: PB4 TIM3 CH1
#define BUZZER_PWM_DRIVER PWMD3
#else
#error "Buzzer has not been defined for selected board"
#endif

    pwmStart(&BUZZER_PWM_DRIVER, &pwm_config);

    pwmChangePeriod(&BUZZER_PWM_DRIVER, 1000000 / 349);
    pwmEnableChannel(&BUZZER_PWM_DRIVER, 0, PWM_PERCENTAGE_TO_WIDTH(&BUZZER_PWM_DRIVER, 5000)); // 50%
}
}

/** @} */