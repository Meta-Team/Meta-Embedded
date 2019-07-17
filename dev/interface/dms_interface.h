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
#include "../vehicle/engineer/vehicle_engineer.h"
#include "chassis_interface.h"

#define ADC_GRP1_NUM_CHANNELS   4
#define ADC_GRP1_BUF_DEPTH      1


/**
 * @name DMSInterface
 * @note DMS stands for "distance measuring sensor"
 * @brief Interface to read the values of distance measuring sensors from the ADC port,
 *        process the analog samples and knowing whether the wheels are hanging.
 * @note about the sensors and the corresponding pins:
 *       FR 0 - ADC1_IN12(PC2) - L1
 *       FL 1 - ADC1_IN13(PC3) -
 *       BL 2 - ADC1_IN14(PC4) -
 *       BR 3 - ADC1_IN15(PC5) -
 */

class DMSInterface {

public:

//    enum DMS_id_t {
//        FR, // front right motor, 0
//        FL, // front left motor, 1
//        BL, // back left motor, 2
//        BR, // back right motor, 3
//    };

    static void init(int sensor_num);

    /**
     * @brief check whether a wheel is hanging by comparing the sample value
     *        with the corresponding value for wheel height.
     * @return true if the wheel of given index is hanging, false if not
     */
    static bool check_hanging(int index);

private:

    static int num;

    static adcsample_t samples[];

    /**
     * @brief return the raw sample value from the DMS at the given index.
     */
    static adcsample_t get_distance(int index);

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
