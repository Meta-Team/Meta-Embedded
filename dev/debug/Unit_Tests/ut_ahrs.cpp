//
// Created by liuzikai on 2019-05-13.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#include "ahrs.h"
#include "buzzer_scheduler.h"

using namespace chibios_rt;

static constexpr Matrix33 AHRS_MATRIX = {{0.0f, 0.0f, 1.0f},
                                         {1.0f, 0.0f, 0.0f},
                                         {0.0f, 1.0f, 0.0f}};



//static constexpr Matrix33 ANGLE_INSTALLATION_MATRIX = {{1.0f, 0.0f, 0.0f},
//                                                              {0.0f, 1.0f, 0.0f},
//                                                              {0.0f, 0.0f, -1.0f}};
//
//
//static constexpr Matrix33 GYRO_INSTALLATION_MATRIX = {{0.0f,  -1.0f, 0.0f},
//                                                             {0.0f,  0.0f,  1.0f},
//                                                             {-1.0f, 0.0f,  0.0f}};

static constexpr Matrix33 ANGLE_INSTALLATION_MATRIX = {{1.0f, 0.0f, 0.0f},
                                                       {0.0f, 1.0f, 0.0f},
                                                       {0.0f, 0.0f, 1.0f}};


static constexpr Matrix33 GYRO_INSTALLATION_MATRIX = {{1.0f,  0.0f, 0.0f},
                                                             {0.0f,  1.0f,  0.0f},
                                                             {0.0f, 0.0f,  1.0f}};
AHRSOnBoard ahrs;

class AHRSFeedbackThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("AHRS");
//        ahrs.load_calibration_data({0.682773649f, -0.682926177f, -0.257317185f});
        ahrs.start(AHRS_MATRIX, HIGHPRIO - 2, HIGHPRIO - 3, HIGHPRIO - 1);
        BuzzerSKD::init(LOWPRIO);
        BuzzerSKD::play_sound(BuzzerSKD::sound_startup);
        while (!shouldTerminate()) {
            Vector3D angle = ahrs.get_angle() * ANGLE_INSTALLATION_MATRIX;
            Shell::printf("!a,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
                          angle.x,
                          angle.y,
                          angle.z);
            Vector3D gyro = ahrs.get_gyro() * GYRO_INSTALLATION_MATRIX;
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


void cmd_echo_gyro_bias(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_bias");
        return;
    }

    chprintf(chp, "gyro_bias.x = %f" SHELL_NEWLINE_STR, ahrs.gyro_bias.x);
    chprintf(chp, "gyro_bias.y = %f" SHELL_NEWLINE_STR, ahrs.gyro_bias.y);
    chprintf(chp, "gyro_bias.z = %f" SHELL_NEWLINE_STR, ahrs.gyro_bias.z);
}


ShellCommand ahrsShellCommands[] = {
        {"echo_bias", cmd_echo_gyro_bias},
        {nullptr,     nullptr}
};

int main(void) {

    halInit();
    System::init();

    Shell::start(NORMALPRIO - 10);
    Shell::addCommands(ahrsShellCommands);
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