//
// Created by liuzikai on 2019-02-09.
// Refactored by Qian Chen on 2019-11-16.
//

/**
 * @file    buzzer.cpp
 * @brief   Interface to control buzzer to alert or to play sounds, including some pre-install sounds.
 *
 * @addtogroup buzzer
 * @{
 */

#include "buzzer_scheduler.h"

constexpr BuzzerSKD::note_with_time_t BuzzerSKD::sound_alert[];
constexpr BuzzerSKD::note_with_time_t BuzzerSKD::sound_startup[];
constexpr BuzzerSKD::note_with_time_t BuzzerSKD::sound_startup_intel[];
constexpr BuzzerSKD::note_with_time_t BuzzerSKD::sound_infinity_warning[];
constexpr BuzzerSKD::note_with_time_t BuzzerSKD::sound_little_star[];
constexpr BuzzerSKD::note_with_time_t BuzzerSKD::sound_orange[];
constexpr BuzzerSKD::note_with_time_t BuzzerSKD::sound_da_bu_zi_duo_ge[];
constexpr BuzzerSKD::note_with_time_t BuzzerSKD::sound_kong_fu_FC[];
constexpr BuzzerSKD::note_with_time_t BuzzerSKD::sound_nyan_cat[];

chibios_rt::ThreadReference BuzzerSKD::skdThreadReference;

bool BuzzerSKD::alerting_ = false;
int BuzzerSKD::last_change_time = -1;

BuzzerSKD::SKDThread BuzzerSKD::skdThread;

void BuzzerSKD::init(tprio_t skd_prio) {
    BuzzerSKD::skdThread.started = true;
    skdThreadReference = BuzzerSKD::skdThread.start(skd_prio);
}

void BuzzerSKD::play_sound(const note_with_time_t sound[]) {
    // Enabled only when it is not alerting.
    if (!alerting_) {
        skdThread.music_header = sound;  // past the pointer into the class
        skdThread.current_pointer= skdThread.music_header;
        last_change_time = -1;  // -1 to let the code identify that it's the first note.
    }
}

void BuzzerSKD::SKDThread::main(void) {
    setName("BuzzerSKD");
    while(!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        if (alerting_) {
            started = false;
            chSchGoSleepS(CH_STATE_SUSPENDED);
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        if (music_header != nullptr) {

            if (last_change_time == -1) {  // The first note situation
                BuzzerIF::change_playing_note(current_pointer->note);
                last_change_time = TIME_I2MS(chibios_rt::System::getTime());
            } else if ((TIME_I2MS(chibios_rt::System::getTime()) - last_change_time)
                       > current_pointer->duration) {  // The following notes situation
                current_pointer++;
                if(current_pointer->note == InfLoop)  {
                    current_pointer = skdThread.music_header + (int)(current_pointer->duration)*2-2;
                } else if (current_pointer->note == Finish) {
                    music_header = nullptr;
                    BuzzerIF::change_playing_note(current_pointer->note);
                } else {  // When finished, set music header as nullptr, disable the code above.
                    BuzzerIF::change_playing_note(current_pointer->note);
                    last_change_time = TIME_I2MS(chibios_rt::System::getTime());
                }
            }
        }
        sleep(TIME_MS2I(BUZZER_SKD_INTERVAL));  // 10ms.

    }
}

void BuzzerSKD::alert_on() {
    alerting_ = true;

    BuzzerIF::change_playing_note(ALERT_NOTE);
}

void BuzzerSKD::alert_off() {
    BuzzerIF::change_playing_note(Silent);
    alerting_ = false;

    // Wake the skdThread up.
    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    if (!skdThread.started) {
        skdThread.started = true;
        chSchWakeupS(skdThreadReference.getInner(), 0);
    }
    chSysUnlock();  /// --- EXIT S-Locked state ---

}

bool BuzzerSKD::alerting() {
    return alerting_;
}

/** @} */