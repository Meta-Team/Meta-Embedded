//
// Created by liuzikai on 2019-01-17.
// Modified by Tony Zhang on 2023/4/15


#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"
#include "referee_interface.h"
#include "printf.h"

// TODO: Fix the bugs in this file. (Shell)
using namespace chibios_rt;
bool enabled_send = false;

class RefereeEchoThread : public BaseStaticThread <2048> {
private:
    void main() final {
        setName("referee_echo");
        while(!shouldTerminate()) {

            LOG("chassis_volt = %u" , (unsigned int) Referee::power_heat.chassis_volt);
            LOG("chassis_current = %u" , (unsigned int) Referee::power_heat.chassis_current);
            LOG("chassis_power = %f" , Referee::power_heat.chassis_power);
            LOG("chassis_power_buffer = %u" , (unsigned int) Referee::power_heat.chassis_power_buffer);
            LOG("shooter_heat0 = %u" , (unsigned int) Referee::power_heat.shooter_id1_17mm_cooling_heat);
            LOG("");

//
            LOG("armor_id = %u" , (unsigned int) Referee::robot_hurt.armor_id);
            LOG("hurt_type = %u" , (unsigned int) Referee::robot_hurt.hurt_type);
            LOG("launch opening status = %u", (unsigned int) Referee::dart_client.dart_launch_opening_status);
            LOG("attack target = %u", (unsigned int) Referee::dart_client.dart_attack_target);
            LOG("target change time = %u", (unsigned int) Referee::dart_client.target_change_time);
            LOG("launch cmd time = %u", (unsigned int) Referee::dart_client.operate_launch_cmd_time);
            LOG("remain_time = %u ",(unsigned int) Referee::ext_dart_remaining_time.dart_remaining_time);

            LOG("");
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
DEF_SHELL_CMD_START(cmd_add_rect)
    (void) argv;
    if (argc != 0) {
        return false;
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

//    Referee::set_graphic(rect);
    Referee::add_tx_graphic(rect);
    enabled_send = true;
    chprintf(chp, "added" SHELL_NEWLINE_STR);
    return true; // command executed successfully
DEF_SHELL_CMD_END


/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
DEF_SHELL_CMD_START(cmd_modify_rect)
    (void) argv;
    if (argc != 0) {
        return false;
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

//    Referee::set_graphic(rect);
    Referee::add_tx_graphic(rect);
    chprintf(chp, "modified" SHELL_NEWLINE_STR);
    return true; // command executed successfully
DEF_SHELL_CMD_END

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
DEF_SHELL_CMD_START(cmd_del_rect)
    (void) argv;
    if (argc != 0) {
        return false;
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

//    Referee::set_graphic(rect);
    Referee::add_tx_graphic(rect);
    enabled_send = false;
    chprintf(chp, "deleted" SHELL_NEWLINE_STR);
    return true; // command executed successfully
DEF_SHELL_CMD_END

// Shell commands to ...
Shell::Command templateShellCommands[] = {
        {"add","add", cmd_add_rect, nullptr},
        {"modify","modify", cmd_modify_rect,nullptr},
        {"delete","delete", cmd_del_rect,nullptr},
        {nullptr,    nullptr,nullptr,nullptr}
};

int main() {
    halInit();
    System::init();
    LED::all_off();

    Shell::addCommands(templateShellCommands);
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);

    Referee::init();
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
