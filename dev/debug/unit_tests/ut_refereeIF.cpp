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
        Referee::init(NORMALPRIO);
        Referee::set_client_number(1, 36);
        bool test = true;
        while(!shouldTerminate()) {

            LOG("chassis_volt = %u" , (unsigned int) Referee::power_heat_data.chassis_volt);
            LOG("chassis_current = %u" , (unsigned int) Referee::power_heat_data.chassis_current);
            LOG("chassis_power = %f" , Referee::power_heat_data.chassis_power);
            LOG("chassis_power_buffer = %u" , (unsigned int) Referee::power_heat_data.chassis_power_buffer);
            LOG("shooter_heat0 = %u" , (unsigned int) Referee::power_heat_data.shooter_heat0);
            LOG("shooter_heat1 = %u" , (unsigned int) Referee::power_heat_data.shooter_heat1);
            LOG("");

            LOG("bullet_type = %u" , (unsigned int) Referee::shoot_data.bullet_type);
            LOG("bullet_freq = %u" , (unsigned int) Referee::shoot_data.bullet_freq);
            LOG("bullet_speed = %u" , (unsigned int) Referee::shoot_data.bullet_speed);
            LOG("");

            LOG("armor_id = %u" , (unsigned int) Referee::robot_hurt.armor_id);
            LOG("hurt_type = %u" , (unsigned int) Referee::robot_hurt.hurt_type);
            LOG("");

            Referee::set_client_light(0, test);
            test = !test;
            Referee::request_to_send(Referee::CLIENT);
            sleep(TIME_MS2I(2000));
        }
    }
} refereeEchoThread;

int main() {
    halInit();
    System::init();
    LED::all_off();

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
