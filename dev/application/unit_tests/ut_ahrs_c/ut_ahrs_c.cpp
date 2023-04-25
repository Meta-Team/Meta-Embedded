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

// Rotation matrix from the IMU frame to compass frame
static constexpr Matrix33 IMU_COMPASS_ROTATION_MATRIX = {{-1.0f, 0.0f, 0.0f},
                                                         {0.0f, -1.0f, 0.0f},
                                                         {0.0f, 0.0f, 1.0f}};

// Rotation matrix from the compass frame to board frame
static constexpr Matrix33 COMPASS_BOARD_ROTATION_MATRIX = {{0.0f,  0.0f, 1.0f},
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

    ahrs.start(IMU_COMPASS_ROTATION_MATRIX, COMPASS_BOARD_ROTATION_MATRIX, NORMALPRIO);

    feedbackThread.start(NORMALPRIO+1);

    // Indicating thread running
    LED::green_on();
    BuzzerSKD::init(LOWPRIO);
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_dji);

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