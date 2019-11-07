//
// Created by 钱晨 on 2019/11/2.
//
#include "ch.hpp"
#include "hal.h"

#include "buzzer.h"
#include "led.h"
#include "common_macro.h"
#include "debug/shell/shell.h"

#include "pid_controller.hpp"
#include "interface/can_interface.h"
#include "interface/chassis_interface.h"
#include "interface/gimbal_interface.h"

unsigned const INTERVAL = 1;
using namespace chibios_rt;

#if defined(BOARD_RM_2018_A)
#else
#error "supports only RM Board 2018 A currently"
#endif

CANInterface can1(&CAND1);

#define CHASSIS_PID_V2I_KP 26.0f
#define CHASSIS_PID_V2I_KI 0.1f
#define CHASSIS_PID_V2I_KD 0.02f
#define CHASSIS_PID_V2I_I_LIMIT 2000.0f
#define CHASSIS_PID_V2I_OUT_LIMIT 6000.0f
#define CHASSIS_PID_V2I_PARAMS \
    {CHASSIS_PID_V2I_KP, CHASSIS_PID_V2I_KI, CHASSIS_PID_V2I_KD, \
    CHASSIS_PID_V2I_I_LIMIT, CHASSIS_PID_V2I_OUT_LIMIT}

float target_velocity = 0.0f;

class CurrentSendThread : public chibios_rt::BaseStaticThread<512> {
    void main() final {

        bool a = false;
        int last_log_time;
        last_log_time = SYSTIME;

        PIDController lv2i;
        PIDController rv2i;

        lv2i.change_parameters(CHASSIS_PID_V2I_PARAMS);
        rv2i.change_parameters(CHASSIS_PID_V2I_PARAMS);

        float last_log_angle;
        LED::all_off();
        while (!shouldTerminate()) {
            LED::led_on(1);
            ChassisIF::target_current[0] = lv2i.calc(ChassisIF::feedback[0].actual_velocity,target_velocity);
            ChassisIF::target_current[1] = rv2i.calc(ChassisIF::feedback[1].actual_velocity,-target_velocity);
            ChassisIF::send_chassis_currents();
        }
        sleep(TIME_MS2I(1));
    }
}currentSendThread;

static void cmd_set_velocity (BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "set_velocity");
        return;
    }
    target_velocity = Shell::atof(argv[0]);
}
ShellCommand CotrollerCommands[] = {
        {"set_velocity",   cmd_set_velocity},
        {nullptr,         nullptr}
};
int main(){

    halInit();
    chibios_rt::System::init();
    Shell::start(HIGHPRIO);
    Shell::addCommands(CotrollerCommands);
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    LED::all_off();
    can1.start(HIGHPRIO - 1);
    LED::led_on(2);
    ChassisIF::init(&can1);
    LED::led_on(3);

    LED::led_on(4);
    currentSendThread.start(HIGHPRIO - 2);
    Buzzer::play_sound(Buzzer::sound_startup_intel, HIGHPRIO - 3);  // Now play the startup sound
    return 0;
}