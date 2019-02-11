//
// Created by liuzikai on 2019-02-09.
//

#include "buzzer.h"
#include "serial_shell.h"

constexpr Buzzer::note_with_time_t Buzzer::sound_startup[];
constexpr Buzzer::note_with_time_t Buzzer::sound_startup_intel[];
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
                        {PWM_OUTPUT_ACTIVE_HIGH, nullptr},
                        {PWM_OUTPUT_DISABLED, nullptr},
                        {PWM_OUTPUT_DISABLED, nullptr},
                        {PWM_OUTPUT_DISABLED, nullptr}
                },
                0,
                0
        };
        pwmStart(&PWMD12, &pwm_config);

        while (sound_seq->note != -1) {

            if (sound_seq->note > 0) {  // is a valid note
//                Shell::printf("Note %d" SHELL_NEWLINE_STR, sound_seq->note);
                // See note above for this formula
                pwmChangePeriod(buzzer_pwm_driver, 1000000 / (unsigned long) sound_seq->note);
                // TODO: find a better method to support multiple board
                pwmEnableChannel(buzzer_pwm_driver, 0, PWM_PERCENTAGE_TO_WIDTH(buzzer_pwm_driver, 5000)); // 50%
            } else {  // is a silent note
                pwmDisableChannel(buzzer_pwm_driver, 0);
            }

            sleep(TIME_MS2I(sound_seq->duration));

            sound_seq++; // move to next note
        }

        pwmStop(buzzer_pwm_driver);
    }
}