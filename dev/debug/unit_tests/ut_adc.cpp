//
// Created by Kerui Zhu on 7/13/2019.
// Modified by
//

#include <interface/buzzer.h>
#include "ch.h"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"

#include "dms_interface.h"

#define FF_SWITCH_PAD GPIOB
#define FFL_SWITCH_PIN_ID GPIOB_PIN0
#define FFR_SWITCH_PIN_ID GPIOB_PIN1
#define SWITCH_TOUCH_PAL_STATUS PAL_HIGH


class ADCEchoThread : public chibios_rt::BaseStaticThread <512> {
private:
    void main() final {
        setName("adc_echo");
        DMSInterface::init(4);
        uint16_t hanging_trigger = 2000;
        uint16_t landed_trigger = 3000;
        while (!shouldTerminate()) {
//             LOG("%u %u %u", DMSInterface::get_raw_sample(DMSInterface::FR), DMSInterface::get_raw_sample(DMSInterface::FL),
//                 DMSInterface::get_raw_sample(DMSInterface::BL));

            if ( DMSInterface::get_raw_sample(DMSInterface::FR) > landed_trigger)
                LOG("FR landed");
            if ( DMSInterface::get_raw_sample(DMSInterface::FL) > landed_trigger)
                LOG("FL landed");

            if ( DMSInterface::get_raw_sample(DMSInterface::FR) < hanging_trigger)
                LOG("FR hanging");
            if ( DMSInterface::get_raw_sample(DMSInterface::FL) < hanging_trigger)
                LOG("FL hanging");

//            if ( DMSInterface::get_raw_sample(DMSInterface::BR) > landed_trigger)
//                LOG("BR landed");
//            if ( DMSInterface::get_raw_sample(DMSInterface::BL) > landed_trigger)
//                LOG("BL landed");
//
//            if ( DMSInterface::get_raw_sample(DMSInterface::BR) < hanging_trigger)
//                LOG("BR hanging");
//            if ( DMSInterface::get_raw_sample(DMSInterface::BL) < hanging_trigger)
//                LOG("BL hanging");

//            if ( palReadPad(FF_SWITCH_PAD, FFL_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS )
//                LOG("FFL reaches stage");
//            if ( palReadPad(FF_SWITCH_PAD, FFR_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS )
//                LOG("FFR reaches stage");

            sleep(TIME_MS2I(250));
        }
    }
} adcEchoThread;

/*
 * Application entry point.
 */
int main(void) {

    halInit();
    chSysInit();

    Shell::start(HIGHPRIO);

    adcEchoThread.start(NORMALPRIO);

    Buzzer::play_sound(Buzzer::sound_startup, NORMALPRIO);

#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(1);
#endif
    return 0;
}
