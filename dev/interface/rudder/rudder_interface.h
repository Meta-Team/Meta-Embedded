//
// Created by Wu Feiyang on 2023/7/11.
//

#ifndef EXPLORER_RUDDER_INTERFACE_H
#define EXPLORER_RUDDER_INTERFACE_H
#include "ch.hpp"
#include "hal.h"
#include "hal_pwm.h"

class Rudder{
public:
    typedef enum{
        MG995,
        AFD30T60MG,
        RUDDER_TYPE_CNT
    }rudderType_t;

    Rudder(PWMDriver *driver,pwmchannel_t channel, rudderType_t rudderType);
    ~Rudder();
    static void init();
    static void start(PWMDriver *driver, PWMConfig *config);
    void enable();
    void disable();
    static void stop(PWMDriver *driver);
    void set_rudder_angle(int angle);

private:
    static constexpr PWMConfig pwm_default_config = {
            10000,    // frequency 10000Hz
            100,        // 10 ms, 1/10000s * 100 = 0.01s = 10ms
            nullptr,
            {
                    {PWM_COMPLEMENTARY_OUTPUT_DISABLED, nullptr},  // it's all CH1 for current support boards
                    {PWM_COMPLEMENTARY_OUTPUT_DISABLED, nullptr},
                    {PWM_COMPLEMENTARY_OUTPUT_DISABLED, nullptr},
                    {PWM_COMPLEMENTARY_OUTPUT_DISABLED, nullptr}
            },
            0,
            0,
    };
    PWMDriver * driver_;
    static PWMConfig config_;
    pwmchannel_t channel_;
    rudderType_t rudderType_;
};

#endif //EXPLORER_RUDDER_INTERFACE_H
