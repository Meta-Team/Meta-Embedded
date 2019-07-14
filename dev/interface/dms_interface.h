//
// Created by Kerui Zhu on 7/14/2019.
//

#ifndef META_INFANTRY_DMS_INTERFACE_H
#define META_INFANTRY_DMS_INTERFACE_H
#include <interface/buzzer.h>
#include "ch.h"
#include "hal.h"
#include "common_macro.h"
#include "led.h"
#include "serial_shell.h"

#define ADC_GRP1_NUM_CHANNELS   4
#define ADC_GRP1_BUF_DEPTH      1

class DMSInterface {

public:

    static void init(int sensor_num);

    static adcsample_t get_distance(int index);

private:

    static int num;

    static adcsample_t samples[];

    static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n);

    static void adcerrorcallback(ADCDriver *adcp, adcerror_t err);

    static constexpr ADCConversionGroup adcgrpcfg = {
            FALSE,
            ADC_GRP1_NUM_CHANNELS,
            adccallback,
            adcerrorcallback,
            0,                        /* CR1 */
            ADC_CR2_SWSTART,          /* CR2 */
            ADC_SMPR1_SMP_AN12(ADC_SAMPLE_56) | ADC_SMPR1_SMP_AN13(ADC_SAMPLE_56) |
            ADC_SMPR1_SMP_AN14(ADC_SAMPLE_56) | ADC_SMPR1_SMP_AN15(ADC_SAMPLE_56),
            0,                        /* SMPR2 */
            ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),                        /* SQR1 */
            0,                        /* SQR2 */
            ADC_SQR3_SQ4_N(ADC_CHANNEL_IN15) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN14) |
            ADC_SQR3_SQ2_N(ADC_CHANNEL_IN13) | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN12)
    };
};

#endif //META_INFANTRY_DMS_INTERFACE_H
