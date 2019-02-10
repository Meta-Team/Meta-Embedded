//
// Created by liuzikai on 2019-02-09.
//

#include "buzzer.h"
#include "serial_shell.h"

PWMConfig Buzzer::buzzer_pwm_config = {
        1000000,
        1000,
        nullptr,
        {
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr},
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr},
                {PWM_OUTPUT_DISABLED, nullptr},
                {PWM_OUTPUT_DISABLED, nullptr}
        },
        0,
        0
};
Buzzer::BuzzerThread Buzzer::buzzerThread;

void Buzzer::play_sound(Buzzer::note_t sound[], int interval_ms, tprio_t prio) {
    buzzerThread.sound_seq = (int *) sound;
    buzzerThread.interval = interval_ms;
    buzzerThread.start(prio);

    Shell::printf("PWMD12.clock = %u" SHELL_NEWLINE_STR, (unsigned int)PWMD12.clock);
}

void Buzzer::BuzzerThread::main(void) {
    setName("buzzer");
    while(*sound_seq != -1) {
        // Frequency = PW
        PWMConfig pwm_config = {
                1000000,

        };
    }
}