//
// Created by Kerui Zhu on 7/13/2019.
//

#include <interface/buzzer.h>
#include "ch.h"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"

#include "dms_interface.h"


class ADCEchoThread : public chibios_rt::BaseStaticThread <512> {
private:
    void main() final {
        setName("adc_echo");
        DMSInterface::init(3);
        while (!shouldTerminate()) {
            LOG("%u %u %u", DMSInterface::get_distance(0), DMSInterface::get_distance(1), DMSInterface::get_distance(2));
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
