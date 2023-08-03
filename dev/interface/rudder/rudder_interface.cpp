//
// Created by Wu Feiyang on 2023/7/11.
//
#include "rudder_interface.h"
PWMConfig Rudder::config_;

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
Rudder::Rudder(PWMDriver *driver, pwmchannel_t channel,rudderType_t rudderType) {
    driver_ = driver;
    channel_ = channel;
    rudderType_ = rudderType;
    config_.channels[channel_].mode = PWM_OUTPUT_ACTIVE_HIGH;
}

Rudder::~Rudder() {
    disable();
}

void Rudder::start(PWMDriver *driver, PWMConfig *config) {
    pwmStart(driver,&config_);
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
void Rudder::stop(PWMDriver *driver) {
    pwmStop(driver);
    for(int i = 0; i< 4; i++){
        if(config_.channels[i].mode == PWM_OUTPUT_ACTIVE_HIGH) {
            pwmDisableChannel(driver,i);
        }
        config_.channels[i].mode = PWM_OUTPUT_DISABLED;
    }
}

void Rudder::init() {
    config_ = pwm_default_config;
}

void Rudder::disable() {
    config_.channels[channel_].mode = PWM_OUTPUT_DISABLED;
}

void Rudder::enable() {
    config_.channels[channel_].mode = PWM_OUTPUT_ACTIVE_HIGH;
}


