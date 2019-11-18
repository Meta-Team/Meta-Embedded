//
// Created by 钱晨 on 2019/11/16.
//

#ifndef META_INFANTRY_BUZZER_INTERFACE_H
#define META_INFANTRY_BUZZER_INTERFACE_H

#include "ch.hpp"
#include "hal.h"

// NOTICE: buzzer pin is all CH1 for current support boards. Otherwise, buzzer.cpp needs revision.
#if defined(BOARD_RM_2018_A)
// RM_BOARD_2018_A: PH6 TIM12 CH1
#define BUZZER_PWM_DRIVER PWMD12
#elif defined(BOARD_RM_2017)
// RM_BOARD_2017: PB4 TIM3 CH1
#define BUZZER_PWM_DRIVER PWMD3
#else
#error "BuzzerSKD interface has not been defined for selected board"
#endif
#define BUZZER_INTERFACE_INTERVAL 10
/**
 * @brief BuzzerIF, Buzzer interface is used to play "now_playing_note".
 *
 * */
class BuzzerIF {
public:
    typedef enum {
        Do1L = 262,     // 261.63Hz
        Re2L = 294,     // 293.66Hz
        Mi3L = 330,     // 329.63Hz
        Fa4L = 349,     // 349.23Hz
        So5L = 392,     // 392.00Hz
        La6L = 440,     // 440.00Hz
        Si7L = 494,     // 493.88Hz

        Do1M = 523,     // 523.25Hz
        Re2M = 587,     // 587.33Hz
        Mi3M = 659,     // 659.26Hz
        Fa4M = 698,     // 698.46Hz
        So5M = 784,     // 784.00Hz
        La6M = 880,     // 880.00Hz
        Si7M = 988,     // 987.77Hz

        Do1H = 1047,     // 1046.50Hz
        Re2H = 1175,     // 1174.66Hz
        Mi3H = 1319,     // 1318.51Hz
        Fa4H = 1397,     // 1396.91Hz
        So5H = 1568,     // 1567.98Hz
        La6H = 1760,     // 1760.00Hz
        Si7H = 1976,     // 1975.53Hz

        Silent = 0,
        Finish = -1,
        InfLoop = -2

    } note_t;

    static void change_playing_note (int note);

private:

    static bool buzzer_idle;        // If the buzzerIF is playing sound. If do, just interrupt it. If not, start PWM.

    static note_t now_playing_note; // Single note to play.

    // PWM configuration for buzzer
    static constexpr PWMConfig pwm_config = {
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

    static constexpr int ALERT_NOTE = Do1M;  // continuous alert playing_note
};


#endif //META_INFANTRY_BUZZER_INTERFACE_H
