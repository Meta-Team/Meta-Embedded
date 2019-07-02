//
// Created by liuzikai on 2019-05-13.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#include "ahrs.h"
#include "buzzer.h"

using namespace chibios_rt;

static constexpr Matrix33 GIMBAL_AHRS_INSTALL_MATRIX = {{ 0.0f,  0.0f,  1.0f},
                                                        { 1.0f,  0.0f,  0.0f},
                                                        { 0.0f,  1.0f,  0.0f}};

AHRSOnBoard ahrs;

class AHRSFeedbackThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("AHRS");
        // TODO: try initialization here
        ahrs.start(GIMBAL_AHRS_INSTALL_MATRIX, HIGHPRIO - 2, HIGHPRIO - 3, HIGHPRIO - 1);
        Buzzer::play_sound(Buzzer::sound_startup, LOWPRIO);
        while (!shouldTerminate()) {
//            Shell::printf("!a,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
//                          ahrs.angle.x,
//                          ahrs.angle.y,
//                          ahrs.angle.z);
            Shell::printf("gyro ,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
                          ahrs.get_gyro().x,
                          ahrs.get_gyro().y,
                          ahrs.get_gyro().z);
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
    chprintf(chp, "gyro_bias.y = %f" SHELL_NEWLINE_STR, ahrs.gyro_bias.x);
    chprintf(chp, "gyro_bias.z = %f" SHELL_NEWLINE_STR, ahrs.gyro_bias.x);
}


ShellCommand ahrsShellCommands[] = {
        {"echo_bias", cmd_echo_gyro_bias},
        {nullptr, nullptr}
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