//
// Created by Tianyi Han on 3/19/2023.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "shell.h"

#include "bmi088_interface.h"
#include "ist8310_interface.h"
#include "ahrs_c_interface.h"

using namespace chibios_rt;

static constexpr Matrix33 IMU_ROTATION_MATRIX = {{-1.0f, 0.0f, 0.0f},
                                                 {0.0f, -1.0f, 0.0f},
                                                 {0.0f, 0.0f, 1.0f}};

static constexpr Matrix33 BOARD_ROTATION_MATRIX = {{0.0f,  0.0f, 1.0f},
                                                   {0.0f,  1.0f,  0.0f},
                                                   {-1.0f, 0.0f,  0.0f}};

AHRSOnBoard_C ahrs;

class AHRSFeedbackThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("AHRS");
        while (!shouldTerminate()) {
/*            Vector3D accel = ahrs.get_accel();
            Shell::printf("!a, %.2f, %.2f, %.2f\t",
                          accel.x,
                          accel.y,
                          accel.z);
            Vector3D gyro = ahrs.get_gyro();
            Shell::printf("!g, %.2f, %.2f, %.2f\t",
                          gyro.x,
                          gyro.y,
                          gyro.z);
            Vector3D compass = ahrs.get_magnet();
            Shell::printf("!m, %.2f, %.2f, %.2f\t",
                          compass.x,
                          compass.y,
                          compass.z);*/
            Vector3D angle = ahrs.get_angle();
            Shell::printf("!angle\t%.2f\t%.2f\t%.2f" ENDL,
                          angle.x,
                          angle.y,
                          angle.z);
            /*Vector3D gyro = GYRO_INSTALLATION_MATRIX * ahrs.get_gyro();
            Shell::printf("gyro ,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
                          gyro.x,
                          gyro.y,
                          gyro.z);
            Shell::printf("gyro.x = %.2f, gyro.z = %.2f, angle.y = %.2f ,ans = %.2f" SHELL_NEWLINE_STR,
                          gyro.x,
                          gyro.z,
                          angle.y,
                          gyro.x * cosf(angle.y / 180.0f * M_PI) + gyro.z * sinf(angle.y / 180.0f * M_PI));*/
            sleep(TIME_MS2I(10));
        }
    }
} feedbackThread;

int main(void) {
    halInit();
    System::init();

    Shell::start(HIGHPRIO);
    // Shell::addCommands(ahrsShellCommands);
    LED::all_off();

    ahrs.start(IMU_ROTATION_MATRIX, BOARD_ROTATION_MATRIX, NORMALPRIO);

    // Indicating thread running
    LED::green_on();
    BuzzerSKD::init(LOWPRIO);
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_dji);

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