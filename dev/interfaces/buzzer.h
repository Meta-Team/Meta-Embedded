//
// Created by liuzikai on 2019-02-09.
//

#ifndef META_INFANTRY_BUZZER_H
#define META_INFANTRY_BUZZER_H

#include "ch.hpp"
#include "hal.h"

// RM_BOARD_2018_A: PH6 TIM12
class Buzzer{

public:

    typedef enum{
        Do1L = 262,     // 261.63Hz    3822us
        Re2L = 294,     // 293.66Hz    3405us
        Mi3L = 330,     // 329.63Hz    3034us
        Fa4L = 349,     // 349.23Hz    2863us
        So5L = 392,     // 392.00Hz    2551us
        La6L = 440,     // 440.00Hz    2272us
        Si7L = 494,     // 493.88Hz    2052us

        Do1M = 523,     // 523.25Hz    1911us
        Re2M = 587,     // 587.33Hz    1703us
        Mi3M = 659,     // 659.26Hz    1517us
        Fa4M = 698,     // 698.46Hz    1432us
        So5M = 784,     // 784.00Hz    1276us
        La6M = 880,     // 880.00Hz    1136us
        Si7M = 988,     // 987.77Hz    1012us

        Do1H = 1047,     // 1046.50Hz   956us
        Re2H = 1175,     // 1174.66Hz   851us
        Mi3H = 1319,     // 1318.51Hz   758us
        Fa4H = 1397,     // 1396.91Hz   716us
        So5H = 1568,     // 1567.98Hz   638us
        La6H = 1760,     // 1760.00Hz   568us
        Si7H = 1976,     // 1975.53Hz   506us

        Silent  = 0,
        Finish  = -1,

    } note_t;

    static void play_sound(const note_t sound[], int interval_ms, tprio_t prio);

    static constexpr note_t sound_startup[] = {
            So5L, Si7L, Re2M, Do1M, Mi3M, So5M, Silent, Finish
    };

    static constexpr note_t sound_startup_intel[] = {
            Fa4H, Fa4H, Silent, So5M, Do1H, So5M, Re2H, Re2H, Re2H, Finish
    };

    static constexpr note_t sound_little_star[] = {
            Do1M, Silent, Do1M, Silent, So5M, Silent, So5M, Silent, La6M, Silent, La6M, Silent, So5M, Silent, Silent,
            Fa4M, Silent, Fa4M, Silent, Mi3M, Silent, Mi3M, Silent, Re2M, Silent, Re2M, Silent, Do1M, Silent, Silent,
            So5M, Silent, So5M, Silent, Fa4M, Silent, Fa4M, Silent, Mi3M, Silent, Mi3M, Silent, Re2M, Silent, Silent,
            So5M, Silent, So5M, Silent, Fa4M, Silent, Fa4M, Silent, Mi3M, Silent, Mi3M, Silent, Re2M, Silent, Silent,
            Do1M, Silent, Do1M, Silent, So5M, Silent, So5M, Silent, La6M, Silent, La6M, Silent, So5M, Silent, Silent,
            Fa4M, Silent, Fa4M, Silent, Mi3M, Silent, Mi3M, Silent, Re2M, Silent, Re2M, Silent, Do1M, Silent, Silent,
            Finish
    };

private:

    static constexpr PWMDriver* buzzer_pwm_driver = &PWMD12;

    class BuzzerThread : public chibios_rt::BaseStaticThread <512> {
    public:

        note_t const* sound_seq;
        int interval;

        BuzzerThread() : sound_seq(nullptr) {};

    private:
        void main(void) final;

    };

    static BuzzerThread buzzerThread;
};


#endif //META_INFANTRY_BUZZER_H
