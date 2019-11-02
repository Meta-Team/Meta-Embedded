//
// Created by 钱晨 on 2019/11/2.
//
#include "ch.hpp"
#include "hal.h"

#include "buzzer.h"
#include "led.h"
#include "common_macro.h"

#include "pid_controller.hpp"
#include "interface/can_interface.h"
#include "interface/chassis_interface.h"
#include "interface/gimbal_interface.h"

using namespace chibios_rt;

#if defined(BOARD_RM_2018_A)
#else
#error "supports only RM Board 2018 A currently"
#endif

#define GIMBAL_PID_YAW_V2I_KP 680.0f
#define GIMBAL_PID_YAW_V2I_KI 0.075f
#define GIMBAL_PID_YAW_V2I_KD 0.0f
#define GIMBAL_PID_YAW_V2I_I_LIMIT 10000.0f
#define GIMBAL_PID_YAW_V2I_OUT_LIMIT 20000.0f
#define GIMBAL_PID_YAW_V2I_PARAMS \
    {GIMBAL_PID_YAW_V2I_KP, GIMBAL_PID_YAW_V2I_KI, GIMBAL_PID_YAW_V2I_KD, \
    GIMBAL_PID_YAW_V2I_I_LIMIT, GIMBAL_PID_YAW_V2I_OUT_LIMIT}

#define GIMBAL_PID_YAW_A2V_KP 12.0f
#define GIMBAL_PID_YAW_A2V_KI 0.0f
#define GIMBAL_PID_YAW_A2V_KD 0.09f
#define GIMBAL_PID_YAW_A2V_I_LIMIT 1440.0f
#define GIMBAL_PID_YAW_A2V_OUT_LIMIT 1440.0f
#define GIMBAL_PID_YAW_A2V_PARAMS \
    {GIMBAL_PID_YAW_A2V_KP, GIMBAL_PID_YAW_A2V_KI, GIMBAL_PID_YAW_A2V_KD, \
    GIMBAL_PID_YAW_A2V_I_LIMIT, GIMBAL_PID_YAW_A2V_OUT_LIMIT}

CANInterface can1(&CAND1);

class CurrentSendThread : public chibios_rt::BaseStaticThread<512> {
    void main() final {

        bool a = false;
        int last_log_time;
        last_log_time = SYSTIME;
        PIDController yawv2i;
        PIDController yawa2v;
        yawv2i.change_parameters(GIMBAL_PID_YAW_V2I_PARAMS);
        yawa2v.change_parameters(GIMBAL_PID_YAW_A2V_PARAMS);
        float last_log_angle;
        LED::all_off();

        while (!shouldTerminate()) {
            LED::led_on(1);
            
            GimbalIF::target_current[0] = yawv2i.calc(GimbalIF::feedback[GimbalIF::YAW].actual_velocity * 1 / 1,yawa2v.calc(GimbalIF::feedback[GimbalIF::YAW].actual_angle,10));
            GimbalIF::send_gimbal_currents();

            if(GimbalIF::feedback[GimbalIF::YAW].actual_velocity == last_log_angle) {
                LED::led_on(8);
            } else {
                LED::led_off(8);
            }
            last_log_angle = GimbalIF::feedback[GimbalIF::YAW].actual_velocity;
            //Shell::printf("%f\n",GimbalIF::feedback[GimbalIF::YAW].actual_velocity);
        }
    }
}currentSendThread;

int main(){

    halInit();
    chibios_rt::System::init();
    Shell::start(HIGHPRIO);

    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    LED::all_off();
    can1.start(HIGHPRIO - 1);
    LED::led_on(2);
    GimbalIF::init(&can1,1,1,(GimbalIF::GM6020),(GimbalIF::RM6623),(GimbalIF::M2006));
    LED::led_on(3);

    LED::led_on(4);
    currentSendThread.start(HIGHPRIO - 2);
    Buzzer::play_sound(Buzzer::sound_startup_intel, HIGHPRIO - 3);  // Now play the startup sound
    return 0;
}