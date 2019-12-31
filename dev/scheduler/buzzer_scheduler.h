//
// Created by liuzikai on 2019-02-09.
// Refactored by Qian Chen on 2019-11-16.
// Thanks a lot to Illini RoboMaster @ UIUC, for playing_note frequency table, sound_startup and sound_little_star in their
// open source project iRM_Embedded_2017.
//
// Thanks a lot to Bruce Helsen (geobruce) Stan Draulans (purewantfun) For their arduino code for nyan cat sound.
// @ https://www.instructables.com/id/Nyan-Cat-on-Arduino/
// Qian Chen using python converted it to the format we used.
//

/**
 * @file    buzzer_scheduler.h
 * @brief   Scheduler to control buzzer to alert or to play sounds, including some pre-install sounds.
 * @usage   Firstly, initialize.                    BuzzerSKD::init(tprio_t skd_prio, tprio_t IF_prio);
 * @usage   Secondly, play sound or play alert.     BuzzerSKD::play_sound(<music>);
 *                                          or      BuzzerSKD::alert_on();
 *                                          or      BuzzerSKD::alert_off();
 * @attention Now the buzzer could be interrupted without cracking. But alert has higher priority than music.
 *            alert_on will interrupt the playing music. If alert_off(), the music continues.
 *            music could also be interrupted by music. Just like what behave a music player.
 * @addtogroup buzzer
 * @{
 */

#ifndef META_INFANTRY_BUZZER_SCHEDULER_H
#define META_INFANTRY_BUZZER_SCHEDULER_H

#include "ch.hpp"
#include "hal.h"
#include "buzzer_interface.h"

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

#define BUZZER_SKD_INTERVAL 10


class BuzzerSKD {

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
     * Structure that represent a note lasting for some time
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
     * Initialize the interface and scheduler
     * @param interface_prio    Buzzer Interface's priority
     * @param skd_prio          Scheduler's priority
     */
    static void init(tprio_t skd_prio);

    /**
     * Play a sound
     * @param sound  An array of note_with_time_t, ending with {Finish(-1), *}
     * @param prio   Priority of the buzzer thread
     */
    static void play_sound(const note_with_time_t sound[]);

    /**
     * Enable continuous alert
     */
    static void alert_on();

    /**
     * Disable continuous alert
     */
    static void alert_off();

    /**
     * Return the status of continuous alert
     * @return The status of continuous alert
     */
    static bool alerting();

private:

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
    public:
        bool started;
        /*
         * Pointer to note_with_time_t array. It's passed into this class at play_sound()
         */
        note_with_time_t const *music_header;
        note_with_time_t const *current_pointer;

        SKDThread() : music_header(nullptr) {};

    private:
        void main(void) final;

    };

    static SKDThread skdThread;

    static int last_change_time;
    static bool alerting_;
    static constexpr int ALERT_NOTE = Do1M;  // continuous alert playing_note
    static chibios_rt::ThreadReference skdThreadReference;

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

    static constexpr note_with_time_t sound_infinity_warning[] {
            {Do1H, 250}, {Do1M, 250}, {InfLoop, 0}
    };
    static constexpr note_with_time_t sound_da_bu_zi_duo_ge[] = {
            //Music bar 1
            {Mi3M, 150}, {Silent, 150}, {Mi3M, 150}, {Silent, 150}, {Mi3M, 150}, {Silent, 150},
            {So5M, 300}, {Silent, 300}, {Do1M, 150}, {Silent, 150}, {Re2M, 300}, {Silent, 300},
            {Fa4M, 150}, {Silent, 150}, {Mi3M, 300}, {Silent, 300},
            //Music bar 2
            {So5M, 150}, {Silent, 150}, {La6M, 150}, {Silent, 150}, {So5M, 150}, {Silent, 150},
            {Fa4M, 150}, {Silent, 150}, {La6M, 300}, {Silent, 300}, {La6M, 150}, {Silent, 150},
            {Si7M, 150}, {Silent, 150}, {La6M, 150}, {Silent, 150}, {So5M, 300}, {Silent, 300},
            //Music bar 3
            {So5M, 150}, {Silent, 150}, {Do1H, 300}, {Silent, 300}, {Si7M, 100}, {Silent,  50},
            {La6M, 100}, {Silent, 150}, {So5M, 300}, {Silent, 300}, {Fa4M, 100}, {Silent,  50},
            {Mi3M, 100}, {Silent, 150}, {La6M, 150}, {Silent, 150}, {Re2M, 150}, {Silent, 150},
            {Mi3M, 150}, {Silent, 150}, {Re2M, 300}, {Silent, 300},
            //Music bar 4
            {Mi3M, 150}, {Silent, 150}, {Mi3M, 150}, {Silent, 150}, {La6M, 150}, {Silent, 150},
            {La6M, 300}, {Silent, 150}, {So5M, 150}, {Silent,  50}, {So5M, 150}, {Silent, 200},
            {So5M, 150}, {Silent, 150}, {So5M, 150}, {Silent, 150}, {Do1H, 150}, {Silent, 150},
            {Si7M, 300}, {Silent, 300},
            //Music bar 5
            {Si7M, 100}, {Silent,  50}, {Do1H, 100}, {Silent, 100}, {Re2H, 100}, {Silent,  50},
            {Do1H, 100}, {Silent,  50}, {Si7M, 100}, {Silent,  50}, {La6M, 100}, {Silent,  50},
            {So5M, 150}, {Silent, 150}, {Re2M, 200}, {Silent, 150}, {La6M, 200}, {Silent, 150},
            {Si7L, 200}, {Silent, 150}, {Do1M, 600}, {Silent, 600},
            //Music bar 6
            {Re2H, 400}, {Silent,  150}, {Re2H, 600}, {Silent, 600}, {Mi3H,600}, {Silent, 600},
            {Do1H, 1200}, {Silent, 150}, {Finish, 150}
    };
    static constexpr note_with_time_t sound_kong_fu_FC[] = {
            {La6H, 150}, {Silent, 150}, {La6H, 100}, {Silent, 50}, {La6H, 100}, {Silent, 100},
            {So5H, 150}, {Silent, 150}, {So5H, 150}, {Silent, 150},{Mi3H, 150}, {Silent, 150},
            {Mi3H, 150}, {Silent, 150}, {Do1H, 300}, {Silent, 300},

            {Re2H, 100}, {Silent,  50}, {Mi3H, 100}, {Silent,  50}, {Re2H, 100}, {Silent,  50},
            {Do1H, 100}, {Silent,  100},{La6M, 150}, {Silent,  150},{Do1H, 150}, {Silent,  150},
            {La6M, 300}, {Silent,  150},{Finish, 150}
    };
    static constexpr note_with_time_t sound_nyan_cat[] = {
            // Intro
            {622, 47}, {Silent, 47}, {659, 47}, {Silent, 47}, {740, 95}, {Silent, 95},
            {988, 95}, {Silent, 95}, {622, 47}, {Silent, 47}, {659, 47}, {Silent, 47},
            {740, 47}, {Silent, 47}, {988, 47}, {Silent, 47}, {1109, 47}, {Silent, 47},
            {1245, 47}, {Silent, 47}, {1109, 47}, {Silent, 47}, {932, 47}, {Silent, 47},
            {988, 95}, {Silent, 95}, {740, 95}, {Silent, 95}, {622, 47}, {Silent, 47},
            {659, 47}, {Silent, 47}, {740, 95}, {Silent, 95}, {988, 95}, {Silent, 95},
            {1109, 47}, {Silent, 47}, {932, 47}, {Silent, 47}, {988, 47}, {Silent, 47},
            {1109, 47}, {Silent, 47}, {1319, 47}, {Silent, 47}, {1245, 47}, {Silent, 47},
            {1319, 47}, {Silent, 47}, {988, 47}, {Silent, 47},
            // Main
            {740, 95}, {Silent, 95}, {831, 95}, {Silent, 95}, {622, 47}, {Silent, 47},
            {622, 47}, {Silent, 47}, {0, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {587, 47}, {Silent, 47}, {554, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {0, 47}, {Silent, 47}, {494, 95}, {Silent, 95}, {554, 95}, {Silent, 95},
            {587, 95}, {Silent, 95}, {587, 47}, {Silent, 47}, {554, 47}, {Silent, 47},
            {494, 47}, {Silent, 47}, {554, 47}, {Silent, 47}, {622, 47}, {Silent, 47},
            {740, 47}, {Silent, 47}, {831, 47}, {Silent, 47}, {622, 47}, {Silent, 47},
            {740, 47}, {Silent, 47}, {554, 47}, {Silent, 47}, {622, 47}, {Silent, 47},
            {494, 47}, {Silent, 47}, {554, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {622, 95}, {Silent, 95}, {740, 95}, {Silent, 95}, {831, 47}, {Silent, 47},
            {622, 47}, {Silent, 47}, {740, 47}, {Silent, 47}, {554, 47}, {Silent, 47},
            {622, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {587, 47}, {Silent, 47},
            {622, 47}, {Silent, 47}, {587, 47}, {Silent, 47}, {554, 47}, {Silent, 47},
            {494, 47}, {Silent, 47}, {554, 47}, {Silent, 47}, {587, 95}, {Silent, 95},
            {494, 47}, {Silent, 47}, {554, 47}, {Silent, 47}, {622, 47}, {Silent, 47},
            {740, 47}, {Silent, 47}, {554, 47}, {Silent, 47}, {622, 47}, {Silent, 47},
            {554, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {554, 95}, {Silent, 95},
            {494, 95}, {Silent, 95}, {554, 95}, {Silent, 95}, {740, 95}, {Silent, 95},
            {831, 95}, {Silent, 95}, {622, 47}, {Silent, 47}, {622, 47}, {Silent, 47},
            {0, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {587, 47}, {Silent, 47},
            {554, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {0, 47}, {Silent, 47},
            {494, 95}, {Silent, 95}, {554, 95}, {Silent, 95}, {587, 95}, {Silent, 95},
            {587, 47}, {Silent, 47}, {554, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {554, 47}, {Silent, 47}, {622, 47}, {Silent, 47}, {740, 47}, {Silent, 47},
            {831, 47}, {Silent, 47}, {622, 47}, {Silent, 47}, {740, 47}, {Silent, 47},
            {554, 47}, {Silent, 47}, {622, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {554, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {622, 95}, {Silent, 95},
            {740, 95}, {Silent, 95}, {831, 47}, {Silent, 47}, {622, 47}, {Silent, 47},
            {740, 47}, {Silent, 47}, {554, 47}, {Silent, 47}, {622, 47}, {Silent, 47},
            {494, 47}, {Silent, 47}, {587, 47}, {Silent, 47}, {622, 47}, {Silent, 47},
            {587, 47}, {Silent, 47}, {554, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {554, 47}, {Silent, 47}, {587, 95}, {Silent, 95}, {494, 47}, {Silent, 47},
            {554, 47}, {Silent, 47}, {622, 47}, {Silent, 47}, {740, 47}, {Silent, 47},
            {554, 47}, {Silent, 47}, {622, 47}, {Silent, 47}, {554, 47}, {Silent, 47},
            {494, 47}, {Silent, 47}, {554, 95}, {Silent, 95}, {494, 95}, {Silent, 95},
            {554, 95}, {Silent, 95}, {494, 95}, {Silent, 95}, {370, 47}, {Silent, 47},
            {415, 47}, {Silent, 47}, {494, 95}, {Silent, 95}, {370, 47}, {Silent, 47},
            {415, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {554, 47}, {Silent, 47},
            {622, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {659, 47}, {Silent, 47},
            {622, 47}, {Silent, 47}, {659, 47}, {Silent, 47}, {740, 47}, {Silent, 47},
            {494, 95}, {Silent, 95}, {494, 95}, {Silent, 95}, {370, 47}, {Silent, 47},
            {415, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {370, 47}, {Silent, 47},
            {659, 47}, {Silent, 47}, {622, 47}, {Silent, 47}, {554, 47}, {Silent, 47},
            {494, 47}, {Silent, 47}, {370, 47}, {Silent, 47}, {311, 47}, {Silent, 47},
            {330, 47}, {Silent, 47}, {370, 47}, {Silent, 47}, {494, 95}, {Silent, 95},
            {370, 47}, {Silent, 47}, {415, 47}, {Silent, 47}, {494, 95}, {Silent, 95},
            {370, 47}, {Silent, 47}, {415, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {494, 47}, {Silent, 47}, {554, 47}, {Silent, 47}, {622, 47}, {Silent, 47},
            {494, 47}, {Silent, 47}, {370, 47}, {Silent, 47}, {415, 47}, {Silent, 47},
            {370, 47}, {Silent, 47}, {494, 95}, {Silent, 95}, {494, 47}, {Silent, 47},
            {466, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {370, 47}, {Silent, 47},
            {415, 47}, {Silent, 47}, {330, 47}, {Silent, 47}, {659, 47}, {Silent, 47},
            {622, 47}, {Silent, 47}, {659, 47}, {Silent, 47}, {740, 47}, {Silent, 47},
            {494, 95}, {Silent, 95}, {466, 95}, {Silent, 95}, {494, 95}, {Silent, 95},
            {370, 47}, {Silent, 47}, {415, 47}, {Silent, 47}, {494, 95}, {Silent, 95},
            {370, 47}, {Silent, 47}, {415, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {554, 47}, {Silent, 47}, {622, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {659, 47}, {Silent, 47}, {622, 47}, {Silent, 47}, {659, 47}, {Silent, 47},
            {740, 47}, {Silent, 47}, {494, 95}, {Silent, 95}, {494, 95}, {Silent, 95},
            {370, 47}, {Silent, 47}, {415, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {370, 47}, {Silent, 47}, {659, 47}, {Silent, 47}, {622, 47}, {Silent, 47},
            {554, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {370, 47}, {Silent, 47},
            {311, 47}, {Silent, 47}, {330, 47}, {Silent, 47}, {370, 47}, {Silent, 47},
            {494, 95}, {Silent, 95}, {370, 47}, {Silent, 47}, {415, 47}, {Silent, 47},
            {494, 95}, {Silent, 95}, {370, 47}, {Silent, 47}, {415, 47}, {Silent, 47},
            {494, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {554, 47}, {Silent, 47},
            {622, 47}, {Silent, 47}, {494, 47}, {Silent, 47}, {370, 47}, {Silent, 47},
            {415, 47}, {Silent, 47}, {370, 47}, {Silent, 47}, {494, 95}, {Silent, 95},
            {494, 47}, {Silent, 47}, {466, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {370, 47}, {Silent, 47}, {415, 47}, {Silent, 47}, {494, 47}, {Silent, 47},
            {659, 47}, {Silent, 47}, {622, 47}, {Silent, 47}, {659, 47}, {Silent, 47},
            {740, 47}, {Silent, 47}, {494, 95}, {Silent, 95}, {554, 95}, {Silent, 47},
            {InfLoop, 26}
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


#endif //META_INFANTRY_BUZZER_SCHEDULER_H

/** @} */