//
// Created by Kerui Zhu on 7/14/2019.
// Modified by LaiXinyi on 7/18/2019.
//

#include "dms_interface.h"

int DMSInterface::num;
adcsample_t DMSInterface::samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
ADCConversionGroup constexpr DMSInterface::adcgrpcfg;

void DMSInterface::init(int sensor_num) {
    num = sensor_num;
    palSetPadMode(GPIOC, GPIOC_PIN2, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, GPIOC_PIN3, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, GPIOC_PIN4, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, GPIOC_PIN5, PAL_MODE_INPUT_ANALOG);
    adcStart(&ADCD1, nullptr);
    adcSTM32EnableTSVREFE();
//    adcStartConversion(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
}

void DMSInterface::get_raw_sample(adcsample_t * sample) {
    adcConvert(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
    for (int i = 0; i < num; i++ ) {
        sample[i] = samples[i];
    }
}

void DMSInterface::adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
    (void) adcp;
    (void) buffer;
    (void) n;
}

void DMSInterface::adcerrorcallback(ADCDriver *adcp, adcerror_t err){
    (void) adcp;
    (void) err;
}