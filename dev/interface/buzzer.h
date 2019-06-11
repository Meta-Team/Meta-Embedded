//
// Created by liuzikai on 2019-02-09.
// Thanks a lot to Illini RoboMaster @ UIUC, for note frequency table, sound_startup and sound_little_star in their
// open source project iRM_Embedded_2017.
//

#ifndef META_INFANTRY_BUZZER_H
#define META_INFANTRY_BUZZER_H

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
#error "Buzzer interface has not been defined for selected board"
#endif


class Buzzer {

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

    /**
     * @brief structure that represent a note lasting for some time
     *
     * @note a sound for buzzer is an array of note_with_time_t, ending with {Finish(-1), *}. To play the sound, use
     *       play_sound(). There are some preset sounds at the bottom of this class definition.
     *
     * @note to make your own sound, you need an array of frequency. You can either write it yourself, or use some tools
     *       to help you. One solution: find a simple MIDI file (if it has multiple tracks, usually it doesn't
     *       sound good if you play only one track.) Then use tools introduced in config/buzzer_sound_array_generator.py
     *       to generate the frequency array.
     */
    struct note_with_time_t {
        int note;  // frequency
        unsigned int duration;  // [ms]
    };

    /**
     * @brief play a sound
     * @param sound  an array of note_with_time_t, ending with {Finish(-1), *}
     * @param prio   priority of the buzzer thread
     */
    static void play_sound(const note_with_time_t sound[], tprio_t prio);

private:

    /**
     * @brief the buzzer thread to play sound
     */
    class BuzzerThread : public chibios_rt::BaseStaticThread<512> {
    public:

        /*
         * Pointer to note_with_time_t array. It's passed into this class at play_sound()
         */
        note_with_time_t const *sound_seq;
        note_with_time_t const *curr;

        BuzzerThread() : sound_seq(nullptr) {};

    private:
        void main(void) final;

    };

    static BuzzerThread buzzerThread;

public:

    static constexpr note_with_time_t sound_alert[] = {
            {Do1M, 300}, {Silent, 100}, {Do1M, 300}, {Silent, 100}, {Do1M, 300}, {Silent, 100}, {Finish, 0}
    };

    static constexpr note_with_time_t sound_startup[] = {
            {So5L, 250}, {Si7L, 250}, {Re2M, 250}, {Do1M, 250}, {Mi3M, 250}, {So5M, 250}, {Silent, 250}, {Finish, 250}
    };

    static constexpr note_with_time_t sound_startup_intel[] = {
            {Fa4H, 250}, {Fa4H, 250}, {Silent, 250}, {So5M, 250}, {Do1H, 250}, {So5M, 250}, {Re2H, 250}, {Re2H, 250},
            {Re2H, 250}, {Finish, 250}
    };

    static constexpr note_with_time_t sound_infinty_warning[] {
            {Do1H, 250}, {Do1M, 250}, {InfLoop, 0}
    };

    static constexpr note_with_time_t sound_little_star[] = {
            {Do1M, 150}, {Silent, 150}, {Do1M, 150}, {Silent, 150}, {So5M, 150}, {Silent, 150},
            {So5M, 150}, {Silent, 150}, {La6M, 150}, {Silent, 150}, {La6M, 150}, {Silent, 150},
            {So5M, 150}, {Silent, 150}, {Silent, 150},
            {Fa4M, 150}, {Silent, 150}, {Fa4M, 150}, {Silent, 150}, {Mi3M, 150}, {Silent, 150},
            {Mi3M, 150}, {Silent, 150}, {Re2M, 150}, {Silent, 150}, {Re2M, 150}, {Silent, 150},
            {Do1M, 150}, {Silent, 150}, {Silent, 150},
            {So5M, 150}, {Silent, 150}, {So5M, 150}, {Silent, 150}, {Fa4M, 150}, {Silent, 150},
            {Fa4M, 150}, {Silent, 150}, {Mi3M, 150}, {Silent, 150}, {Mi3M, 150}, {Silent, 150},
            {Re2M, 150}, {Silent, 150}, {Silent, 150},
            {So5M, 150}, {Silent, 150}, {So5M, 150}, {Silent, 150}, {Fa4M, 150}, {Silent, 150},
            {Fa4M, 150}, {Silent, 150}, {Mi3M, 150}, {Silent, 150}, {Mi3M, 150}, {Silent, 150},
            {Re2M, 150}, {Silent, 150}, {Silent, 150},
            {Do1M, 150}, {Silent, 150}, {Do1M, 150}, {Silent, 150}, {So5M, 150}, {Silent, 150},
            {So5M, 150}, {Silent, 150}, {La6M, 150}, {Silent, 150}, {La6M, 150}, {Silent, 150},
            {So5M, 150}, {Silent, 150}, {Silent, 150},
            {Fa4M, 150}, {Silent, 150}, {Fa4M, 150}, {Silent, 150}, {Mi3M, 150}, {Silent, 150},
            {Mi3M, 150}, {Silent, 150}, {Re2M, 150}, {Silent, 150}, {Re2M, 150}, {Silent, 150},
            {Do1M, 150}, {Silent, 150}, {Silent, 150}, {Finish, 150}
    };

    static constexpr note_with_time_t sound_orange[] = {
            { 880, 1230}, {   0,   66}, {1174,  204}, {   0,   12}, {1046,  410}, {   0,   22}, { 932,  410},
            {   0,   22}, { 880,  410}, {   0,   22}, { 698,  204}, {   0,   12}, { 783,  204}, {   0,  228},
            { 698,  820}, {   0,   44}, { 659,  204}, {   0,   12}, { 698,  204}, {   0,   12}, { 783,  204},
            {   0,   12}, { 880,  410}, {   0,   23}, { 783,  410}, {   0,   23}, { 698,  410}, {   0,   23},
            { 783,  410}, {   0,   23}, { 880,  204}, {   0,   12}, {1046, 1230}, {   0,   66}, {1174,  204},
            {   0,   12}, {1046,  410}, {   0,   23}, { 932,  410}, {   0,   23}, { 880,  410}, {   0,   23},
            { 698,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 523,  204}, {   0,   12}, { 698,  820},
            {   0,   44}, { 659,  204}, {   0,   12}, { 698,  204}, {   0,   12}, { 659,  204}, {   0,   12},
            { 587, 1846}, {   0,  530}, { 880,  204}, {   0,   12}, { 932,  204}, {   0,   12}, { 880,  204},
            {   0,   12}, { 698,  615}, {   0,   33}, { 880,  204}, {   0,   12}, { 932,  204}, {   0,   12},
            { 880,  204}, {   0,   12}, { 698,  820}, {   0,  692}, { 880,  204}, {   0,   12}, { 932,  204},
            {   0,   12}, { 880,  204}, {   0,   12}, { 698,  409}, {   0,   22}, { 698,  204}, {   0,   12},
            { 880,  204}, {   0,   12}, {1046,  409}, {   0,   22}, {1046, 1025}, {   0,  487}, { 880,  204},
            {   0,   12}, { 932,  204}, {   0,   12}, { 880,  204}, {   0,   12}, { 698,  615}, {   0,   33},
            { 880,  204}, {   0,   12}, { 932,  204}, {   0,   12}, { 880,  204}, {   0,   12}, { 698,  820},
            {   0,  692}, { 880,  204}, {   0,   12}, { 932,  204}, {   0,   12}, { 880,  204}, {   0,   12},
            { 698,  409}, {   0,   22}, { 698,  204}, {   0,   12}, {1046,  215}, {   0,  217}, { 932,  215},
            {   0,  217}, { 880,  215}, {   0,  217}, { 783,  215}, {   0,  649}, { 587,  204}, {   0,   12},
            { 698,  204}, {   0,   12}, { 698,  204}, {   0,   12}, { 783,  410}, {   0,   22}, { 587,  409},
            {   0,   22}, { 698,  204}, {   0,   12}, { 698,  204}, {   0,   12}, { 783,  204}, {   0, 1308},
            { 587,  204}, {   0,   12}, { 698,  204}, {   0,   12}, { 698,  204}, {   0,   12}, { 783,  409},
            {   0,   22}, { 880,  409}, {   0,   22}, { 698,  204}, {   0,   12}, { 698,  204}, {   0,   12},
            { 698,  204}, {   0, 1308}, { 587,  409}, {   0,   22}, { 698,  204}, {   0,   12}, { 783,  107},
            {   0,  325}, { 587,  409}, {   0,   22}, { 698,  107}, {   0,  325}, { 659,  409}, { 783,  107},
            {   0,  541}, { 880,  204}, {   0,   12}, { 698, 1435}, {   0,   77}, { 880,  204}, {   0,   12},
            { 783, 1435}, {   0,   77}, {1046,  409}, {   0,   22}, {1046,  820}, {   0,   44}, { 932,  204},
            {   0,   12}, { 880,  409}, {   0,   22}, { 783, 1435}, {   0,  941}, { 698,  409}, {   0,   22},
            { 783,  409}, {   0,   22}, { 698,  409}, {   0,   22}, { 880,  204}, {   0,   12}, { 880,  204},
            {   0,   12}, { 783,  204}, {   0,   12}, { 880,  204}, {   0,  228}, { 880,  204}, {   0,   12},
            { 783,  204}, {   0,   12}, { 880,  204}, {   0,  228}, { 880,  204}, {   0,   12}, { 783,  204},
            {   0,   12}, { 880,  204}, {   0,   12}, {1046,  204}, {   0,   12}, { 880,  204}, {   0,   12},
            { 783,  204}, {   0,   12}, { 659,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 783,  204},
            {   0,   12}, { 698,  204}, {   0,   12}, { 783,  204}, {   0,  228}, { 783,  204}, {   0,   12},
            { 698,  204}, {   0,   12}, { 783,  204}, {   0,  228}, { 880,  204}, {   0,   12}, { 932,  204},
            {   0,   12}, { 880,  409}, {   0,   22}, { 698,  204}, {   0,   12}, { 783,  204}, {   0,   12},
            { 698,  204}, {   0,   12}, { 880,  204}, {   0,   12}, { 880,  204}, {   0,   12}, { 783,  204},
            {   0,   12}, { 880,  204}, {   0,  228}, { 880,  204}, {   0,   12}, { 783,  204}, {   0,   12},
            { 880,  204}, {   0,  228}, { 880,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 880,  204},
            {   0,   12}, {1046,  204}, {   0,   12}, { 880,  204}, {   0,   12}, { 783,  204}, {   0,   12},
            { 659,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 698,  204},
            {   0,   12}, { 783,  204}, {   0,  228}, {1046,  409}, {   0,   23}, {1046,  615}, {   0, 1329},
            { 880,  204}, {   0,   12}, { 880,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 880,  204},
            {   0,  228}, { 880,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 880,  204}, {   0,  228},
            { 880,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 880,  204}, {   0,   12}, {1046,  204},
            {   0,   12}, { 880,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 698,  204}, {   0,   12},
            { 783,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 698,  204}, {   0,   12}, { 783,  204},
            {   0,  228}, { 783,  204}, {   0,   12}, { 698,  204}, {   0,   12}, { 783,  204}, {   0,  228},
            { 880,  204}, {   0,   12}, { 932,  204}, {   0,   12}, { 880,  409}, {   0,   23}, { 698,  204},
            {   0,   12}, { 783,  204}, {   0,   12}, { 698,  204}, {   0,   12}, { 880,  204}, {   0,   12},
            { 880,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 880,  204}, {   0,  228}, { 880,  204},
            {   0,   12}, { 783,  204}, {   0,   12}, { 880,  204}, {   0,  228}, { 880,  204}, {   0,   12},
            { 783,  204}, {   0,   12}, { 880,  204}, {   0,   12}, {1046,  204}, {   0,   12}, { 880,  204},
            {   0,   12}, { 783,  204}, {   0,   12}, { 659,  204}, {   0,   12}, { 783,  204}, {   0,   12},
            { 783,  204}, {   0,   12}, { 698,  204}, {   0,   12}, { 783,  204}, {   0,  228}, {1046,  409},
            {   0,   23}, {1046, 1025}, /*{ 587,  820}, { 698,  820}, {   0,   44}, { 880,  204}, {   0,   12},
            { 698,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 698, 1846}, {   0,  962}, { 880,  204},
            {   0,   12}, { 698,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 698, 1846}, {   0,  962},
            { 880,  204}, {   0,   12}, { 698,  204}, {   0,   12}, { 783,  204}, {   0,   12}, { 698, 1846},
            {   0,  962}, { 880,  204}, {   0,   12}, { 932,  204}, {   0,   12}, { 880,  204}, {   0,   12},
            { 698, 3488},*/ {  -1,    0}
    };
};


#endif //META_INFANTRY_BUZZER_H
