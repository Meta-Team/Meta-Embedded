//
// Created by Yuan Jinsong on 2019-07-30.
// Modified by Yuan Jinosng on 2019-07-30
//

#include "ch.hpp"
#include "hal.h"
#include "led.h"
#include "debug/shell/shell.h"
#include "common_macro.h"
#include "buzzer.h"

#include "engineer_elevator_logic.h"

ShellCommand sensorsCommands[] = {
        {nullptr, nullptr}
};

class FeedbackThread : public chibios_rt::BaseStaticThread<512> {
    void main() final {
        setName("sensors_fb");
        while(!shouldTerminate()) {
            if (palReadPad(FF_SWITCH_PAD, FFL_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS) {
                Buzzer::play_sound(Buzzer::sound_alert, LOWPRIO);
            }
            if (palReadPad(FF_SWITCH_PAD, FFR_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS) {
                Buzzer::play_sound(Buzzer::sound_alert, LOWPRIO);
            }
            sleep(TIME_MS2I(200));
        }
    }
} feedbackThread;

int main(void) {
    halInit();
    chibios_rt::System::init();

    Shell::start(HIGHPRIO);
    Shell::addCommands(sensorsCommands);

    LED::red_off();
    LED::green_off();

    feedbackThread.start(NORMALPRIO);

    Buzzer::play_sound(Buzzer::sound_startup, LOWPRIO);

#if CH_CFG_NO_IDLE_THREAD // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled,  main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(1);
#endif
    return 0;
}