//
// Created by Wu Feiyang on 2023/7/11.
//

#ifndef META_RUDDER_INTERFACE_H
#define META_RUDDER_INTERFACE_H
#include "ch.hpp"
#include "hal.h"
#include "hal_pwm.h"

class Rudder{
public:
    typedef enum{
        MG995,
        RUDDER_TYPE_CNT
    }rudderType_t;

    Rudder(PWMDriver* driver,PWMConfig* config,pwmchannel_t channel, rudderType_t rudderType);
    ~Rudder();
    void start();
    void stop();
    void set_rudder_angle(int angle);

private:
    PWMConfig pwm_default_config = {
            10000,    // frequency 10000Hz
            100,        // 10 ms, 1/10000s * 100 = 0.01s = 10ms
            nullptr,
            {
                    {PWM_OUTPUT_ACTIVE_HIGH, nullptr},  // it's all CH1 for current support boards
                    {PWM_COMPLEMENTARY_OUTPUT_DISABLED, nullptr},
                    {PWM_COMPLEMENTARY_OUTPUT_DISABLED, nullptr},
                    {PWM_COMPLEMENTARY_OUTPUT_DISABLED, nullptr}
            },
            0,
            0
    };
    PWMDriver * driver_;
    PWMConfig * config_;
    pwmchannel_t channel_;
    rudderType_t rudderType_;
};

#endif //META_RUDDER_INTERFACE_H
