//
// Created by Wu Feiyang on 2023/7/11.
//
#include "rudder_interface.h"


/**
 * @brief   Constructor of the rudder
 *
 * @param[in] driver    pointer to a @p PWMDriver object
 * @param[in] config    pointer to a @p PWMConfig object
 * @param[in] channel   PWM channel of the rudder. Note that 0 stand for channel 1 and vice versa.
 * @param[in] rudderType type of the rudder
 *
 * @api
 */
Rudder::Rudder(PWMDriver *driver, PWMConfig *config, pwmchannel_t channel,rudderType_t rudderType) {
    if(config == nullptr && rudderType == MG995){
        config_ = &mg995_pwm_default_config;
    }else if(config == nullptr && rudderType == AFD30T60MG){
        config_ = &afd30t60mg_default_config;
    }
    driver_ = driver;
    channel_ = channel;
    rudderType_ = rudderType;
}

Rudder::~Rudder() {
    pwmStop(driver_);
}
void Rudder::start() {
    config_->channels[channel_].mode = PWM_OUTPUT_ACTIVE_HIGH;
    pwmStart(driver_,config_);
}

void Rudder::set_rudder_angle(int angle) {
    int percentage = 0;
    if(rudderType_ == MG995){
        /**
         * MG995: 0.5ms - 0 degrees
         *        1.0ms - 45 degrees
         *        1.5ms - 90 degrees
         *        2.5ms - 180 degrees
         */
        // 180 - 2000
        percentage = angle * 2000 / 180 + 500;
    }else if(rudderType_ == AFD30T60MG){
        /**
        *
        */
        // 120 - 4000
        angle = angle * 120 / 150; // I don't know what is wrong, but the angle needs remapping.
        percentage = angle * 4000 / 120 + 3000;
    }
    pwmEnableChannel(driver_,channel_,PWM_PERCENTAGE_TO_WIDTH(driver_, percentage));
}
void Rudder::stop() {
    pwmDisableChannel(driver_,channel_);
    pwmStop(driver_);
    config_->channels[channel_].mode = PWM_COMPLEMENTARY_OUTPUT_DISABLED;
}


