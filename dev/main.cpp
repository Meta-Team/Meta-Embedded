//
// Created by 钱晨 on 10/23/20.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
// Other headers here

#include "buzzer_scheduler.h"

using namespace chibios_rt;

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_buzzer(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "buzzer (0-7)");
        return;
    }
    int songsID = Shell::atoi(argv[0]);
    switch (songsID) {
        case 0:
            Shell::printf("Playing sound_alert" SHELL_NEWLINE_STR);
            BuzzerSKD::play_sound(BuzzerSKD::sound_alert);
            break;
        case 1:
            Shell::printf("Playing sound_startup" SHELL_NEWLINE_STR);
            BuzzerSKD::play_sound(BuzzerSKD::sound_startup);
            break;
        case 2:
            Shell::printf("Playing sound_startup_intel" SHELL_NEWLINE_STR);
            BuzzerSKD::play_sound(BuzzerSKD::sound_startup_intel);
            break;
        case 3:
            Shell::printf("Playing sound_orange" SHELL_NEWLINE_STR);
            BuzzerSKD::play_sound(BuzzerSKD::sound_orange);
            break;
        case 4:
            Shell::printf("Playing sound_da_bu_zi_duo_ge" SHELL_NEWLINE_STR);
            BuzzerSKD::play_sound(BuzzerSKD::sound_da_bu_zi_duo_ge);
            break;
        case 5:
            Shell::printf("Playing sound_kong_fu_FC" SHELL_NEWLINE_STR);
            BuzzerSKD::play_sound(BuzzerSKD::sound_kong_fu_FC);
            break;
        case 6:
            Shell::printf("Playing sound_nyan_cat" SHELL_NEWLINE_STR);
            BuzzerSKD::play_sound(BuzzerSKD::sound_nyan_cat);
            break;
        case 7:
            Shell::printf("Playing sound_little_star" SHELL_NEWLINE_STR);
            BuzzerSKD::play_sound(BuzzerSKD::sound_little_star);
            break;
        default:
            shellUsage(chp, "buzzer (0-7)");
    }
}


// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"buzzer", cmd_buzzer},
        {nullptr,    nullptr}
};


// Thread to ...
class SkywalkerAdjustThread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("Blink");
        bool led_on = false;
        unsigned int long time_stamp = SYSTIME;  // [ms] get current time.
        unsigned int long  time_interval = 1000; // [ms]
        while (!shouldTerminate()) {
            if(SYSTIME - time_stamp > time_interval) {
                if(led_on){
                    LED::led_off(1);
                } else {
                    LED::led_on(1);
                }
                time_stamp = SYSTIME;
                led_on = !led_on;
            }
            sleep(TIME_MS2I(100));
        }
    }
} blinkThread;


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);

    BuzzerSKD::init(NORMALPRIO);

    blinkThread.start(NORMALPRIO + 1);


#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}

