//
// Created by liuzikai on 2019-06-07.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"

#include "interface/ahrs/mpu6500.h"
#include "interface/ahrs/ist8310.h"

using namespace chibios_rt;

MPUOnBoard mpu6500;
ISTOnBoard ist8310;

class IST8310FeedbackThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("ist8310");
        mpu6500.start(HIGHPRIO - 1);
        ist8310.start(HIGHPRIO - 2);
        while (!shouldTerminate()) {
            Shell::printf("%.4f, %.4f, %.4f" SHELL_NEWLINE_STR,
                          ist8310.magnet.x, ist8310.magnet.y, ist8310.magnet.z);
            sleep(TIME_MS2I(100));
        }
    }
} feedbackThread;

int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    LED::green_off();
    LED::red_off();

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