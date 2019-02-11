//
// Created by liuzikai on 2019-02-09.
//

#include "buzzer.h"
#include "serial_shell.h"

constexpr Buzzer::note_t Buzzer::sound_startup[];
constexpr Buzzer::note_t Buzzer::sound_startup_intel[];
constexpr Buzzer::note_t Buzzer::sound_little_star[];


Buzzer::BuzzerThread Buzzer::buzzerThread;

void Buzzer::play_sound(const Buzzer::note_t sound[], int interval_ms, tprio_t prio) {
    buzzerThread.sound_seq = sound;
    buzzerThread.interval = interval_ms;
    buzzerThread.start(prio);
}

void Buzzer::BuzzerThread::main(void) {
    setName("buzzer");

    if (sound_seq != nullptr) {

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
        Shell::printf("PWMD12.clock = %u" SHELL_NEWLINE_STR, (unsigned int) PWMD12.clock);

        while (*sound_seq != -1) {

            if (*sound_seq > 0) {  // is a valid note
//                Shell::printf("Note %d" SHELL_NEWLINE_STR, (int) *sound_seq);
                // Frequency = PW
                pwmChangePeriod(buzzer_pwm_driver, 1000000 / (unsigned long) (*sound_seq));
                // TODO: find a better method to support multiple board
                pwmEnableChannel(buzzer_pwm_driver, 0, PWM_PERCENTAGE_TO_WIDTH(buzzer_pwm_driver, 5000)); // 50%
            } else {  // is a silent note
                pwmDisableChannel(buzzer_pwm_driver, 0);
            }

            sleep(TIME_MS2I(interval));

            sound_seq++; // move to next note
        }

        pwmStop(buzzer_pwm_driver);
    }
}