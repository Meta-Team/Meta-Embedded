//
// Created by Kerui Zhu on 7/13/2019.
// Modified by
//

#include <scheduler/buzzer_scheduler.h>
#include "ch.h"
#include "hal.h"

#include "led.h"
#include "shell.h"

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
//        uint16_t hanging_trigger = 1500;
//        uint16_t landed_trigger = 2500;
        while (!shouldTerminate()) {
            adcsample_t data[4];
            DMSInterface::get_raw_sample(data);
            LOG("%u %u %u %u", data[DMSInterface::FR], data[DMSInterface::FL],
                 data[DMSInterface::BL], data[DMSInterface::BR]);

//            unsigned print = 9000000;
//
//            if ( DMSInterface::get_raw_sample(DMSInterface::FR) > landed_trigger)           print += 1000;
//            else if ( DMSInterface::get_raw_sample(DMSInterface::FR) < hanging_trigger )    print += 2000;
//
//            if ( DMSInterface::get_raw_sample(DMSInterface::FL) > landed_trigger)           print += 100;
//            else if ( DMSInterface::get_raw_sample(DMSInterface::FL) < hanging_trigger )    print += 200;
//
//            if ( DMSInterface::get_raw_sample(DMSInterface::BR) > landed_trigger)           print += 10;
//            else if ( DMSInterface::get_raw_sample(DMSInterface::BR) < hanging_trigger )    print += 20;
//
//            if ( DMSInterface::get_raw_sample(DMSInterface::BL) > landed_trigger)           print += 1;
//            else if ( DMSInterface::get_raw_sample(DMSInterface::BL) < hanging_trigger )    print += 2;
//
//            if ( palReadPad(FF_SWITCH_PAD, FFL_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS )  print += 100000;
//            if ( palReadPad(FF_SWITCH_PAD, FFR_SWITCH_PIN_ID) == SWITCH_TOUCH_PAL_STATUS )  print += 10000;
//
//            LOG("FFL|FFR|FR|FL|BL|BR 1reach 2hanging 1landed 0 %u", print);

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
    BuzzerSKD::init(NORMALPRIO+1);

    adcEchoThread.start(NORMALPRIO);

    BuzzerSKD::play_sound(BuzzerSKD::sound_startup);

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
