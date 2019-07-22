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
    palSetPadMode(GPIOC, GPIOC_ADC1_IN12, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, GPIOC_ADC1_IN13, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, GPIOC_ADC1_IN14, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, GPIOC_ADC1_IN15, PAL_MODE_INPUT_ANALOG);
    adcStart(&ADCD1, nullptr);
    adcSTM32EnableTSVREFE();
}

adcsample_t DMSInterface::get_raw_sample(DMS_id_t id) {
    adcConvert(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
    return samples[id];
}

void DMSInterface::adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
    (void) adcp;
}

void DMSInterface::adcerrorcallback(ADCDriver *adcp, adcerror_t err){
    (void) adcp;
    (void) err;
}