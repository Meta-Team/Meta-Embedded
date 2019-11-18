//
// Created by 钱晨 on 2019/11/16.
//

#include "buzzer_interface.h"


bool BuzzerIF::buzzer_idle       = true;

constexpr PWMConfig BuzzerIF::pwm_config;

BuzzerIF::note_t BuzzerIF::now_playing_note = Finish;


void BuzzerIF::change_playing_note(int note) {
    now_playing_note = (note_t) note;
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
}