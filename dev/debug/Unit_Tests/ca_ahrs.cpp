//
// Created by liuzikai on 2019-07-13.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "shell.h"

#include "ahrs.h"
#include "sd_card_interface.h"

#if defined(INFANTRY)
#include "vehicle_infantry.h"
#elif defined(HERO)
#include "vehicle_hero.h"
#endif

// Depends on the install direction of the board
#define ON_BOARD_AHRS_MATRIX2 {{0.0f, -1.0f, 0.0f}, \
                              {1.0f, 0.0f, 0.0f}, \
                              {0.0f, 0.0f, 1.0f}}

#define GIMBAL_ANGLE_INSTALLATION_MATRIX {{1.0f, 0.0f, 0.0f}, \
                                          {0.0f, 1.0f, 0.0f}, \
                                          {0.0f, 0.0f, -1.0f}}


#define GIMBAL_GYRO_INSTALLATION_MATRIX {{0.0f,  -1.0f, 0.0f}, \
                                         {0.0f,  0.0f,  1.0f}, \
                                         {-1.0f, 0.0f,  0.0f}}

AHRSOnBoard ahrs;

Vector3D bias_sum;
int bias_count = 0;

Vector3D last_bias;

static void cmd_erase(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "erase");
        return;
    }
    last_bias = bias_sum = Vector3D();
    bias_count = 0;
    chprintf(chp, "SDCard::erase() = %d" SHELL_NEWLINE_STR, SDCard::erase());
}

static void cmd_sd_echo(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "sd_echo");
        return;
    }
    chprintf(chp, "SDCard::read_all() = %d" SHELL_NEWLINE_STR, SDCard::read_all());

    Vector3D bias;
    chprintf(chp, "SDCard::get_data() = %d" SHELL_NEWLINE_STR, SDCard::get_data(MPU6500_BIAS_DATA_ID, &bias, sizeof(bias)));
    chprintf(chp, "bias.x = %f" SHELL_NEWLINE_STR, bias.x);
    chprintf(chp, "bias.y = %f" SHELL_NEWLINE_STR, bias.y);
    chprintf(chp, "bias.z = %f" SHELL_NEWLINE_STR, bias.z);
}

class AHRSCalibrationThread : public chibios_rt::BaseStaticThread <512> {
private:
    void main() final {
        setName("mpu6500_ca");
        while (!shouldTerminate()) {
            if (last_bias != ahrs.gyro_bias) {

                LOG("New bias (%f, %f, %f)", ahrs.gyro_bias.x, ahrs.gyro_bias.y, ahrs.gyro_bias.z);

                bias_sum = bias_sum + ahrs.gyro_bias;
                bias_count++;

                Vector3D avg_bias = bias_sum / bias_count;
                LOG("Avg bias (%f, %f, %f) for %d samples", avg_bias.x, avg_bias.y, avg_bias.z, bias_count);

                LOG("SDCard::write_data() = %d" SHELL_NEWLINE_STR, SDCard::write_data(MPU6500_BIAS_DATA_ID, &avg_bias, sizeof(avg_bias)));

                last_bias = ahrs.gyro_bias;
            }
            sleep(TIME_MS2I(100));
        }
    }
} ahrsCalibrationThread;

ShellCommand sdCommands[] = {
        {"erase", cmd_erase},
        {"sd_echo", cmd_sd_echo},
        {nullptr,    nullptr}
};

static const Matrix33 ON_BOARD_AHRS_MATRIX_ = ON_BOARD_AHRS_MATRIX2;

int main(void) {
    halInit();
    chibios_rt::System::init();
    LED::all_off();
    BuzzerSKD::init(LOWPRIO-1);

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(sdCommands);

    LOG("SDCard::init() = %d", SDCard::init());
    SDCard::read_all();

    ahrs.start(ON_BOARD_AHRS_MATRIX_, HIGHPRIO - 2, HIGHPRIO - 3, HIGHPRIO - 1);
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup);

    ahrsCalibrationThread.start(NORMALPRIO + 1);


#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(1);
#endif
    return 0;
}
