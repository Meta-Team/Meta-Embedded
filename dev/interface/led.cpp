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
#include "buzzer_scheduler.h"

extern "C" {
/**
 * @note This function maybe called in very complicated situations, such as system locked states. Be sure to prepare for
 *       them, instead of inducing more complicated error.
 */
void set_led_when_halt(void) {

    /// Turn on red LED when halt
    LED_PAD_ON(GPIOF, GPIOE_LED_RED);    // LED_RED_ON
    LED_PAD_OFF(GPIOF, GPIOF_LED_GREEN); // LED_GREEN_OFF



    // Now system is unlocked

    PWMConfig pwm_config = {
            1000000,
            1000000, // Default playing_note: 1Hz
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

    if ((BUZZER_PWM_DRIVER.state != PWM_STOP) && (BUZZER_PWM_DRIVER.state != PWM_READY)) {

        // Unlock system if in system locked state
        // The following code is extract from chSysUnconditionalUnlock() with debug check eliminated
        if (!port_irq_enabled(port_get_irq_status())) {  // is in locked state
            _stats_stop_measure_crit_thd();
            port_unlock();
        }
        // Hopefully no content switch will happen here...

        pwmStart(&BUZZER_PWM_DRIVER, &pwm_config);  // require unlock state

        chSysUnconditionalLock();  // recover

    }  // if PWM has already started, do nothing

    pwmChangePeriod(&BUZZER_PWM_DRIVER, 1000000 / 349);
    pwmEnableChannel(&BUZZER_PWM_DRIVER, 0, PWM_PERCENTAGE_TO_WIDTH(&BUZZER_PWM_DRIVER, 5000)); // 50%

    chSysUnconditionalLock();  // lock in following infinity loop in chSysHalt()
}
}

/** @} */