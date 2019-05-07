//
// Created by liuzikai on 2019-02-09.
//

#include "buzzer.h"
#include "serial_shell.h"

constexpr Buzzer::note_with_time_t Buzzer::sound_alert[];
constexpr Buzzer::note_with_time_t Buzzer::sound_startup[];
constexpr Buzzer::note_with_time_t Buzzer::sound_startup_intel[];
constexpr Buzzer::note_with_time_t Buzzer::sound_infinty_warning[];
constexpr Buzzer::note_with_time_t Buzzer::sound_little_star[];
constexpr Buzzer::note_with_time_t Buzzer::sound_orange[];

Buzzer::BuzzerThread Buzzer::buzzerThread;

void Buzzer::play_sound(const note_with_time_t sound[], tprio_t prio) {
    buzzerThread.sound_seq = sound; // past the pointer into the class
    buzzerThread.start(prio);
}

void Buzzer::BuzzerThread::main(void) {
    setName("buzzer");

    if (sound_seq != nullptr) {


        /**
         * @note actual PWM frequency = PWM TIM clock frequency/ ((stm32_tim_t.PSC + 1) * (stm32_tim_t.ARR + 1).
         *
         *       At pwmStart():
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
        pwmStart(&BUZZER_PWM_DRIVER, &pwm_config);

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