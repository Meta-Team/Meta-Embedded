//
// Created by liuzikai on 2019-01-17.
//


#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"
#include "referee_interface.h"

using namespace chibios_rt;
bool enabled_send = false;

class RefereeEchoThread : public BaseStaticThread <2048> {
private:
    void main() final {
        setName("referee_echo");
        while(!shouldTerminate()) {

//            LOG("chassis_volt = %u" , (unsigned int) Referee::power_heat_data.chassis_volt);
//            LOG("chassis_current = %u" , (unsigned int) Referee::power_heat_data.chassis_current);
//            LOG("chassis_power = %f" , Referee::power_heat_data.chassis_power);
//            LOG("chassis_power_buffer = %u" , (unsigned int) Referee::power_heat_data.chassis_power_buffer);
//            LOG("shooter_heat0 = %u" , (unsigned int) Referee::power_heat_data.shooter_id1_17mm_cooling_heat);
//            LOG("");
//
//            LOG("bullet_type = %u" , (unsigned int) Referee::shoot_data.bullet_type);
//            LOG("bullet_freq = %u" , (unsigned int) Referee::shoot_data.bullet_freq);
//            LOG("bullet_speed = %u" , (unsigned int) Referee::shoot_data.bullet_speed);
//            LOG("");
//
//            LOG("armor_id = %u" , (unsigned int) Referee::robot_hurt.armor_id);
//            LOG("hurt_type = %u" , (unsigned int) Referee::robot_hurt.hurt_type);
//            LOG("");
            sleep(TIME_MS2I(200));
        }
    }
} refereeEchoThread;

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_add_rect(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "add");
        return;
    }
    Referee::graphic_data_struct_t rect{};

    rect.graphic_name[0] = 'A';
    rect.graphic_name[1] = 'B';
    rect.graphic_name[2] = 'C';

    rect.operate_type = 1;
    rect.graphic_type = 2;
    rect.layer = 1;

    rect.color = 1;

    rect.start_x = 960;
    rect.start_y = 540;

    rect.width = 10;

    rect.radius = 200;

    Referee::set_graphic(rect);
    enabled_send = true;
    chprintf(chp, "added" SHELL_NEWLINE_STR);
}


/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_modify_rect(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "add");
        return;
    }
    Referee::graphic_data_struct_t rect{};
    rect.graphic_name[0] = 'A';
    rect.graphic_name[1] = 'B';
    rect.graphic_name[2] = 'C';

    rect.operate_type = 2;
    rect.graphic_type = 2;
    rect.layer = 1;

    rect.color = 1;

    rect.start_x = 960;
    rect.start_y = 540;

    rect.width = 10;

    rect.radius = 200;

    Referee::set_graphic(rect);
    chprintf(chp, "added" SHELL_NEWLINE_STR);
}

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_del_rect(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "add");
        return;
    }
    Referee::graphic_data_struct_t rect{};

    rect.graphic_name[0] = 'A';
    rect.graphic_name[1] = 'B';
    rect.graphic_name[2] = 'C';

    rect.operate_type = 3;
    rect.graphic_type = 2;
    rect.layer = 1;

    rect.color = 1;

    rect.start_x = 960;
    rect.start_y = 540;

    rect.width = 10;

    rect.radius = 200;

    Referee::set_graphic(rect);
    enabled_send = false;
    chprintf(chp, "added" SHELL_NEWLINE_STR);
}

// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"add", cmd_add_rect},
        {"modify", cmd_modify_rect},
        {"delete", cmd_del_rect},
        {nullptr,    nullptr}
};

int main() {
    halInit();
    System::init();
    LED::all_off();

    Shell::addCommands(templateShellCommands);
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);

    Referee::init(NORMALPRIO);
    refereeEchoThread.start(NORMALPRIO+1);

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
