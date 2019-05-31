//
// Created by liuzikai on 2019-01-17.
//


#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"
#include "referee_interface.h"

using namespace chibios_rt;

class RefereeEchoThread : public BaseStaticThread <2048> {
private:
    void main() final {
        setName("referee_echo");
        Referee::init();
        while(!shouldTerminate()) {

            Shell::printf("chassis_volt = %u" SHELL_NEWLINE_STR, (unsigned int) Referee::power_heat_data.chassis_volt);
            Shell::printf("chassis_current = %u" SHELL_NEWLINE_STR, (unsigned int) Referee::power_heat_data.chassis_current);
            Shell::printf("chassis_power = %f" SHELL_NEWLINE_STR, Referee::power_heat_data.chassis_power);
            Shell::printf("chassis_power_buffer = %u" SHELL_NEWLINE_STR, (unsigned int) Referee::power_heat_data.chassis_power_buffer);
            Shell::printf("shooter_heat0 = %u" SHELL_NEWLINE_STR, (unsigned int) Referee::power_heat_data.shooter_heat0);
            Shell::printf("shooter_heat1 = %u" SHELL_NEWLINE_STR, (unsigned int) Referee::power_heat_data.shooter_heat1);
            Shell::printf(SHELL_NEWLINE_STR);

            Shell::printf("bullet_type = %u" SHELL_NEWLINE_STR, (unsigned int) Referee::shoot_data.bullet_type);
            Shell::printf("bullet_freq = %u" SHELL_NEWLINE_STR, (unsigned int) Referee::shoot_data.bullet_freq);
            Shell::printf("bullet_speed = %u" SHELL_NEWLINE_STR, (unsigned int) Referee::shoot_data.bullet_speed);
            Shell::printf(SHELL_NEWLINE_STR);

            Shell::printf("armor_id = %u" SHELL_NEWLINE_STR, (unsigned int) Referee::robot_hurt.armor_id);
            Shell::printf("hurt_type = %u" SHELL_NEWLINE_STR, (unsigned int) Referee::robot_hurt.hurt_type);
            Shell::printf(SHELL_NEWLINE_STR);

            sleep(TIME_MS2I(200));
        }
    }
} refereeEchoThread;

int main() {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);

    refereeEchoThread.start(NORMALPRIO);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow
        // enter an infinite loop, so we set the priority to lowest
        // before quitting, to let other threads run normally
        BaseThread::setPriority(1);
#endif
    return 0;
}
