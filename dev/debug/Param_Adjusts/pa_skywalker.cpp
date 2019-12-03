//
// Created by ... on YYYY/MM/DD.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "remote_interpreter.h"
#include "buzzer_scheduler.h"
#include "gimbal_interface.h"

using namespace chibios_rt;


// Thread to adjust params of Skywalker-20A for friction wheels
class SkywalkerAdjustThread : public BaseStaticThread <512> {
private:

    static constexpr PWMDriver *FRICTION_WHEEL_PWM_DRIVER = &PWMD8;

    enum friction_wheel_channel_t {
        FW_LEFT = 0,  // The left friction wheel, PI5, channel 0
        FW_RIGHT = 1  // The right friction wheel, PI6, channel 1
    };

    static constexpr PWMConfig FRICTION_WHEELS_PWM_CFG = {
            50000,   // frequency
            1000,    // period
            nullptr, // callback
            {
                    {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH0
                    {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH1
                    {PWM_OUTPUT_DISABLED, nullptr},    // CH2
                    {PWM_OUTPUT_DISABLED, nullptr}     // CH3
            },
            0,
            0
    };

    void main() final {
        setName("pa_skywalker");


        pwmStart(FRICTION_WHEEL_PWM_DRIVER, &FRICTION_WHEELS_PWM_CFG);

        bool has_set = false;

        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::S_DOWN) {
                    pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_RIGHT,
                                     PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 0 * 500 + 500));
                    pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_LEFT,
                                     PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 0 * 500 + 500));
                    LED::red_on();
                    LED::green_off();
            } else if (Remote::rc.s1 == Remote::S_UP) {
                pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_RIGHT,
                                 PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 1 * 500 + 500));
                pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_LEFT,
                                 PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 1 * 500 + 500));
                LED::green_on();
                LED::red_off();

            }

            sleep(TIME_MS2I(100));
        }
    }
} skywalkerAdjustThread;


PWMConfig constexpr SkywalkerAdjustThread::FRICTION_WHEELS_PWM_CFG;

int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    BuzzerSKD::init(LOWPRIO);
    Remote::start();
    chThdSleepMilliseconds(1000);

    skywalkerAdjustThread.start(NORMALPRIO + 1);

    BuzzerSKD::play_sound(BuzzerSKD::sound_startup);


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
