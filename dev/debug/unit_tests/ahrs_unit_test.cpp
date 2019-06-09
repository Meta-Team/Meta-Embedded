//
// Created by liuzikai on 2019-05-13.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"

#include "ahrs.h"

using namespace chibios_rt;

class AHRSFeedbackThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("ahrs");
        MPU6500::start(HIGHPRIO - 1);
        IST8310::start(HIGHPRIO - 2);
        sleep(TIME_MS2I(1000));
        AHRS::start(HIGHPRIO - 3);
        while (!shouldTerminate()) {
            Shell::printf("!a,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
                          AHRS::angle[0] * 57.3f,
                          AHRS::angle[1] * 57.3f,
                          AHRS::angle[2] * 57.3f);
            sleep(TIME_MS2I(300));
        }
    }
} feedbackThread;

int main(void) {

    halInit();
    System::init();

    Shell::start(HIGHPRIO);
    LED::all_off();

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