//
// Created by liuzikai on 2019-01-15.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"

#include "interface/ahrs/mpu6500.h"
using namespace chibios_rt;

MPUOnBoard mpu6500;

class MPU6500FeedbackThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("MPU6500");
        mpu6500.start(HIGHPRIO - 2);
        while (!shouldTerminate()) {
            Shell::printf("w = (%.4f, %.4f, %.4f), a = (%.4f, %.4f, %.4f), temp = %.4f" SHELL_NEWLINE_STR,
                          mpu6500.gyro.x, mpu6500.gyro.y, mpu6500.gyro.z,
                          mpu6500.accel.x, mpu6500.accel.y, mpu6500.accel.z,
                          mpu6500.temperature);
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