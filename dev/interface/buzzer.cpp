//
// Created by liuzikai on 2019-02-09.
//

/**
 * @file    buzzer.cpp
 * @brief   Interface to control buzzer to alert or to play sounds, including some pre-install sounds.
 *
 * @addtogroup buzzer
 * @{
 */

#include "buzzer.h"

constexpr Buzzer::note_with_time_t Buzzer::sound_alert[];
constexpr Buzzer::note_with_time_t Buzzer::sound_startup[];
constexpr Buzzer::note_with_time_t Buzzer::sound_startup_intel[];
constexpr Buzzer::note_with_time_t Buzzer::sound_infinity_warning[];
constexpr Buzzer::note_with_time_t Buzzer::sound_little_star[];
constexpr Buzzer::note_with_time_t Buzzer::sound_orange[];
constexpr Buzzer::note_with_time_t Buzzer::sound_da_bu_zi_duo_ge[];
constexpr Buzzer::note_with_time_t Buzzer::sound_kong_fu_FC[];

constexpr PWMConfig Buzzer::pwm_config;
bool Buzzer::alerting_ = false;

Buzzer::BuzzerThread Buzzer::buzzerThread;

void Buzzer::play_sound(const note_with_time_t sound[], tprio_t prio) {
    buzzerThread.sound_seq = sound; // past the pointer into the class
    buzzerThread.start(prio);
}

void Buzzer::BuzzerThread::main(void) {
    setName("Buzzer");

    if (sound_seq != nullptr) {


        /**
         * @note actual PWM frequency = PWM TIM clock frequency/ ((stm32_tim_t.PSC + 1) * (stm32_tim_t.ARR + 1).
         *
         *       At pwmStart(): (see pwm_config)
         *          stm32_tim_t.PSC is set as (PWM TIM clock / PWMConfig.frequency) - 1 (hal_pwm_lld.c:560)
         *          stm32_tim_t.ARR is set to PWMConfig.period - 1; (hal_pwm_lld.c:565)
         *
         *       So actual PWM frequency = PWMConfig.frequency / PWMConfig.period
         *
         *       But PWM clock frequency must be multiples of PWMConfig.frequency (hal_pwm_lld.c:561).
         *
         *       So here we fix PWMConfig.frequency to 1000000 (TIM12 clock frequency is 168000000), and set
         *       PWMConfig.period to 1000000 / sound_seq->note, to get actual frequency sound_seq->note.
         */

        // Activate PWM driver
        pwmStart(&BUZZER_PWM_DRIVER, &pwm_config);

        alerting_ = false;

        curr = sound_seq;
        while (curr->note != Finish) {

            if (curr->note > 0) {  // a valid note
                // See note above for this formula
                pwmChangePeriod(&BUZZER_PWM_DRIVER, 1000000 / (unsigned long) curr->note);
                pwmEnableChannel(&BUZZER_PWM_DRIVER, 0, PWM_PERCENTAGE_TO_WIDTH(&BUZZER_PWM_DRIVER, 5000)); // 50%
            } else if (curr->note == Silent) {  // a silent note
                pwmDisableChannel(&BUZZER_PWM_DRIVER, 0);
            } else if (curr->note == InfLoop) {  // restart the sound
                curr = sound_seq;
            }

            sleep(TIME_MS2I(curr->duration));

            curr++;  // move to next note
        }

        pwmStop(&BUZZER_PWM_DRIVER);
    }
}

void Buzzer::alert_on() {
    alerting_ = true;
    pwmStart(&BUZZER_PWM_DRIVER, &pwm_config);
    pwmChangePeriod(&BUZZER_PWM_DRIVER, 1000000 / (unsigned long) ALERT_NOTE);
    pwmEnableChannel(&BUZZER_PWM_DRIVER, 0, PWM_PERCENTAGE_TO_WIDTH(&BUZZER_PWM_DRIVER, 5000)); // 50%
}

void Buzzer::alert_off() {
    pwmStop(&BUZZER_PWM_DRIVER);
    alerting_ = false;
}

bool Buzzer::alerting() {
    return alerting_;
}

/** @} */