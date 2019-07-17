//
// Created by Kerui Zhu on 7/14/2019.
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

adcsample_t DMSInterface::get_distance(int index) {
    adcConvert(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
    VAL_CROP(index, num - 1, 0)
    return samples[index];
}

bool DMSInterface::check_hanging(int index) {
    return get_distance(index) < WHEEL_HEIGHT_DMS_VAL;
    // TODO maybe need an interval???
}

void DMSInterface::adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
    (void) adcp;
}

void DMSInterface::adcerrorcallback(ADCDriver *adcp, adcerror_t err){
    (void) adcp;
    (void) err;
}