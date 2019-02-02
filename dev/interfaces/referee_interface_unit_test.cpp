//
// Created by liuzikai on 2019-01-17.
//


#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "referee_interface.h"

using namespace chibios_rt;

class RefereeEchoThread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("referee_echo");
        RefereeSystem::start();
        while(!shouldTerminate()) {

            Shell::printf("Remain Time = %u" SHELL_NEWLINE_STR, (unsigned int) RefereeSystem::gameInfo.remainTime);
            Shell::printf("Remain Life = %u" SHELL_NEWLINE_STR, (unsigned int) RefereeSystem::gameInfo.remainLifeValue);
            Shell::printf("Chassis Out V = %f" SHELL_NEWLINE_STR, RefereeSystem::gameInfo.realChassisOutV);
            Shell::printf("Chassis Out A = %f" SHELL_NEWLINE_STR, RefereeSystem::gameInfo.realChassisOutA);
            Shell::printf("Loc flag = %u" SHELL_NEWLINE_STR, (unsigned int) RefereeSystem::gameInfo.locData.flag);
            Shell::printf("Loc x = %f" SHELL_NEWLINE_STR, RefereeSystem::gameInfo.locData.x);
            Shell::printf("Loc y = %f" SHELL_NEWLINE_STR, RefereeSystem::gameInfo.locData.y);
            Shell::printf("Loc z = %f" SHELL_NEWLINE_STR, RefereeSystem::gameInfo.locData.z);
            Shell::printf("Loc compass = %f" SHELL_NEWLINE_STR, RefereeSystem::gameInfo.locData.compass);
            Shell::printf("Remain Power = %f" SHELL_NEWLINE_STR, RefereeSystem::gameInfo.remainPower);
            Shell::printf(SHELL_NEWLINE_STR);

            Shell::printf("Blood Change Armor ID = %u" SHELL_NEWLINE_STR, (unsigned int) RefereeSystem::realBloodChangedData.hitArmorID);
            Shell::printf("Blood Change Way = %u" SHELL_NEWLINE_STR, (unsigned int) RefereeSystem::realBloodChangedData.way);
            Shell::printf("Blood Change Value = %u" SHELL_NEWLINE_STR, (unsigned int) RefereeSystem::realBloodChangedData.value);
            Shell::printf(SHELL_NEWLINE_STR);

            Shell::printf("Shoot Bullet Speed = %f" SHELL_NEWLINE_STR, RefereeSystem::realShootData.realBulletShootSpeed);
            Shell::printf("Shoot Bullet Freq = %f" SHELL_NEWLINE_STR, RefereeSystem::realShootData.realBulletShootFreq);
            Shell::printf("Shoot Golf Speed = %f" SHELL_NEWLINE_STR, RefereeSystem::realShootData.realGolfShootSpeed);
            Shell::printf("Shoot Golf Freq = %f" SHELL_NEWLINE_STR, RefereeSystem::realShootData.realGolfShootFreq);
            Shell::printf(SHELL_NEWLINE_STR);

            sleep(TIME_MS2I(2000));
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
