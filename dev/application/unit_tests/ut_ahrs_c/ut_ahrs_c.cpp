//
// Created by Tianyi Han on 3/19/2023.
//

#include "ut_ahrs_c.h"

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#include "bmi088_interface.h"
#include "ist8310_interface.h"
#include "ahrs_c_interface.h"

using namespace chibios_rt;

static constexpr Matrix33 MPU_ROTATION_MATRIX = {{-1.0f, 0.0f, 0.0f},
                                                 {0.0f, -1.0f, 0.0f},
                                                 {0.0f, 0.0f, 1.0f}};

static constexpr Matrix33 ANGLE_INSTALLATION_MATRIX = {{1.0f, 0.0f, 0.0f},
                                                       {0.0f, 1.0f, 0.0f},
                                                       {0.0f, 0.0f, 1.0f}};


static constexpr Matrix33 GYRO_INSTALLATION_MATRIX = {{1.0f,  0.0f, 0.0f},
                                                      {0.0f,  1.0f,  0.0f},
                                                      {0.0f, 0.0f,  1.0f}};

BMI088Interface ahrs_imu;
IST8310Interface ahrs_compass;
AHRSOnBoard_C ahrs;

class AHRSFeedbackThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("AHRS");
//        ahrs.load_calibration_data({0.682773649f, -0.682926177f, -0.257317185f});
        while (!shouldTerminate()) {
            Vector3D accel = ahrs_imu.get_accel();
            Shell::printf("!a\t%.4f\t%.4f\t%.4f\t" ,
                          accel.x,
                          accel.y,
                          accel.z);
            Vector3D gyro = ahrs_imu.get_gyro();
            Shell::printf("!g\t%.4f\t%.4f\t%.4f" ENDL,
                          gyro.x,
                          gyro.y,
                          gyro.z);
/*            Vector3D compass = ahrs_compass.get_compass();
            Shell::printf("!g\t%.4f\t%.4f\t%.4f" ENDL,
                          compass.x,
                          compass.y,
                          compass.z);*/
//            Vector3D angle = ANGLE_INSTALLATION_MATRIX * abstract_ahrs -> get_angle();
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
    // Shell::addCommands(ahrsShellCommands);
    LED::all_off();

    chThdSleepMilliseconds(5000);
    // ahrs_imu.start(NORMALPRIO);
    // ahrs_compass.start(NORMALPRIO-1);
    ahrs.start(MPU_ROTATION_MATRIX, NORMALPRIO);

    // Indicating thread running
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