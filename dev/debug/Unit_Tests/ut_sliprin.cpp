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
#include "can_interface.h"
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

float target_velocity = 1.0f;

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
//        LED::all_off();
        while (!shouldTerminate()) {
            LED::led_on(1);
            ChassisIF::target_current[0] = (int) lv2i.calc(ChassisIF::feedback[0].actual_velocity,target_velocity);
            ChassisIF::target_current[1] = (int) rv2i.calc(-ChassisIF::feedback[1].actual_velocity,-target_velocity);
            ChassisIF::send_chassis_currents();
        }
        sleep(TIME_MS2I(1));
    }
}currentSendThread;

//static void cmd_set_velocity (BaseSequentialStream *chp, int argc, char *argv[]) {
//    (void) argv;
//    if (argc != 1) {
//        shellUsage(chp, "set_velocity");
//        return;
//    }
//    target_velocity = Shell::atof(argv[0]);
//}
//ShellCommand ControllerCommands[] = {
//        {"set_velocity",   cmd_set_velocity},
//        {nullptr,         nullptr}
//};
int main(){

    halInit();
    chibios_rt::System::init();
//    Shell::start(HIGHPRIO);
//    Shell::addCommands(ControllerCommands);
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    LED::all_off();
    can1.start(HIGHPRIO - 1);
    ChassisIF::init(&can1);
    time_msecs_t t1 = SYSTIME;
    while (WITHIN_RECENT_TIME(t1, 100)) {
        if (WITHIN_RECENT_TIME(can1.last_error_time, 5)) {  // can error occurs
            t1 = SYSTIME;  // reset the counter
        }
    }
    LED::led_on(1);
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        if (not WITHIN_RECENT_TIME(ChassisIF::feedback[ChassisIF::FR].last_update_time, 5)) {
            // No feedback in last 5 ms (normal 1 ms)
            LOG_ERR("Startup - Chassis FR offline.");
            t = SYSTIME;  // reset the counter
        }
    }
    LED::led_on(2);

    currentSendThread.start(HIGHPRIO - 2);
    Buzzer::play_sound(Buzzer::sound_startup_intel, HIGHPRIO - 3);  // Now play the startup sound
    return 0;
}