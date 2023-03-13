//
// Created by 钱晨 on 2019/11/16.
//

#include "buzzer_interface.h"


bool BuzzerIF::buzzer_idle = true;

constexpr PWMConfig BuzzerIF::pwm_config;

BuzzerIF::note_t BuzzerIF::now_playing_note = Finish;


void BuzzerIF::change_playing_note(int note) {
    now_playing_note = (note_t) note;
    if (now_playing_note > 0) {  // A valid playing_note
        if (buzzer_idle) {
            // Activate PWM driver
            pwmStart(&BUZZER_PWM_DRIVER, &pwm_config);
            buzzer_idle = false;
        }
        // See playing_note above for this formula
        pwmChangePeriod(&BUZZER_PWM_DRIVER, 1000000 / (unsigned long) now_playing_note);
#if defined(BOARD_RM_2018_A) || defined(BOARD_RM_2017)
        pwmEnableChannel(&BUZZER_PWM_DRIVER, 0, PWM_PERCENTAGE_TO_WIDTH(&BUZZER_PWM_DRIVER, 5000)); // 50%
#elif defined(BOARD_RM_C)
        pwmEnableChannel(&BUZZER_PWM_DRIVER, 2, PWM_PERCENTAGE_TO_WIDTH(&BUZZER_PWM_DRIVER, 5000)); // 50%
#endif
    } else if (now_playing_note == Silent) {  // A silent playing_note
        if(!buzzer_idle) {
#if defined(BOARD_RM_2018_A) || defined(BOARD_RM_2017)
            pwmDisableChannel(&BUZZER_PWM_DRIVER, 0);
#elif defined(BOARD_RM_C)
            pwmDisableChannel(&BUZZER_PWM_DRIVER, 2);
#endif
        }
    } else if (now_playing_note == Finish) {  // With the sound finished, the buzzer is idled.
        if(!buzzer_idle){  // Prevent stop PWM repeatedly.
            pwmStop(&BUZZER_PWM_DRIVER);
            buzzer_idle = true;
        }
    }
}