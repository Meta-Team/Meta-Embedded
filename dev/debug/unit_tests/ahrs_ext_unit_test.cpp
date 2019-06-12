//
// Created by liuzikai on 2019-05-13.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#include "ahrs_ext.h"
#include "buzzer.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);
AHRSExt ahrs;

class AHRSFeedbackThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("ahrs_ext");
        ahrs.start(&can1);
        Buzzer::play_sound(Buzzer::sound_startup, LOWPRIO);
        while (!shouldTerminate()) {
            Shell::printf("!a,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
                          ahrs.angle.x,
                          ahrs.angle.y,
                          ahrs.angle.z);
//            Shell::printf("w = (%.4f, %.4f, %.4f), a = (%.4f, %.4f, %.4f)" SHELL_NEWLINE_STR,
//                          ahrs.gyro.x, ahrs.gyro.y, ahrs.gyro.z,
//                          ahrs.accel.x, ahrs.accel.y, ahrs.accel.z);
            sleep(TIME_MS2I(100));
        }
    }
} feedbackThread;

int main(void) {

    halInit();
    System::init();

    Shell::start(NORMALPRIO - 10);
    LED::all_off();

    can1.start(HIGHPRIO);
    feedbackThread.start(NORMALPRIO);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}