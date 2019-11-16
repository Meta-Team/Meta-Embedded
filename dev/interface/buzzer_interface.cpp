//
// Created by 钱晨 on 2019/11/16.
//

#include "buzzer_interface.h"

BuzzerIF::IFThread BuzzerIF::ifThread;

bool BuzzerIF::buzzer_idle       = true;
bool BuzzerIF::play_note_changed = false;

constexpr PWMConfig BuzzerIF::pwm_config;

BuzzerIF::note_t BuzzerIF::now_playing_note = Finish;

void BuzzerIF::init(tprio_t prio) {
    ifThread.start(prio);
}

void BuzzerIF::change_playing_note(int note) {
    now_playing_note = (note_t) note;
    play_note_changed = true;
}

void BuzzerIF::IFThread::main() {
    setName("Buzzer_Interface");
    while(!shouldTerminate()) {
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

        if (play_note_changed) {
            if (now_playing_note > 0) {  // a valid playing_note
                if (buzzer_idle) {
                    // Activate PWM driver
                    pwmStart(&BUZZER_PWM_DRIVER, &pwm_config);
                    buzzer_idle = false;
                }
                // See playing_note above for this formula
                pwmChangePeriod(&BUZZER_PWM_DRIVER, 1000000 / (unsigned long) now_playing_note);
                pwmEnableChannel(&BUZZER_PWM_DRIVER, 0, PWM_PERCENTAGE_TO_WIDTH(&BUZZER_PWM_DRIVER, 5000)); // 50%
            } else if (now_playing_note == Silent) {  // a silent playing_note
                pwmDisableChannel(&BUZZER_PWM_DRIVER, 0);
            } else if (now_playing_note == Finish) {  // with the sound finished, the buzzer is idled.
                pwmStop(&BUZZER_PWM_DRIVER);
                buzzer_idle = true;
            }
            play_note_changed = false; // Prevent it changing the PWM output continuously.
        }
        sleep(TIME_MS2I(BUZZER_INTERFACE_INTERVAL)); // sleep for 10 ms, which may be the lowest frequency we change the note.
    }


}