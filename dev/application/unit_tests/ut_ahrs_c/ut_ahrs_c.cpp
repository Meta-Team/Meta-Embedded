//
// Created by Tianyi Han on 3/19/2023.
//

#include "ut_ahrs_c.h"

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#include "bmi088_interface.h"

using namespace chibios_rt;

BMI088Interface ahrs_c;

class AHRSFeedbackThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("AHRS");
//        ahrs.load_calibration_data({0.682773649f, -0.682926177f, -0.257317185f});
//        ahrs.start(AHRS_MATRIX, HIGHPRIO - 2);
//        BuzzerSKD::init(LOWPRIO);
//        BuzzerSKD::play_sound(BuzzerSKD::sound_startup);
        while (!shouldTerminate()) {
//            Vector3D angle = ANGLE_INSTALLATION_MATRIX * abstract_ahrs -> get_angle();
            Vector3D accel = ahrs_c.get_accel();
//            Shell::printf("!a,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
//                          accel.x,
//                          accel.y,
//                          accel.z);
            Vector3D gyro = ahrs_c.get_gyro();
//            Shell::printf("!g,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
//                          gyro.x,
//                          gyro.y,
//                          gyro.z);
//            Vector3D gyro = GYRO_INSTALLATION_MATRIX * abstract_ahrs -> get_gyro();
//            Shell::printf("gyro ,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
//                          gyro.x,
//                          gyro.y,
//                          gyro.z);
//            Shell::printf("gyro.x = %.2f, gyro.z = %.2f, angle.y = %.2f ,ans = %.2f" SHELL_NEWLINE_STR,
//                          gyro.x,
//                          gyro.z,
//                          angle.y,
//                          gyro.x * cosf(angle.y / 180.0f * M_PI) + gyro.z * sinf(angle.y / 180.0f * M_PI));
            sleep(TIME_MS2I(100));
        }
    }
} feedbackThread;

int main(void) {

    halInit();
    System::init();

    Shell::start(HIGHPRIO);
//    Shell::addCommands(ahrsShellCommands);
    LED::all_off();

    ahrs_c.start(NORMALPRIO);

    // green LED on, indicating thread running
    LED::green_on();

    feedbackThread.start(NORMALPRIO+1);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}