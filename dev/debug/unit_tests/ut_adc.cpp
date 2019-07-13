//
// Created by Kerui Zhu on 7/13/2019.
//

#include <interface/buzzer.h>
#include "ch.h"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"

#define ADC_GRP1_NUM_CHANNELS   4
#define ADC_GRP1_BUF_DEPTH      1

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

/*
 * ADC streaming callback.
 */
size_t nx = 0, ny = 0;

static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
    (void) adcp;
    ny += n;
}

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

    (void) adcp;
    (void) err;
    LED::red_toggle();
}

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    IN11.
 */
static const ADCConversionGroup adcgrpcfg1 = {
        FALSE,
        ADC_GRP1_NUM_CHANNELS,
        NULL,
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

class ADCEchoThread : public chibios_rt::BaseStaticThread <512> {
private:
    void main() final {
        setName("adc_echo");
        while (!shouldTerminate()) {
            adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
            LOG("%u %u %u %u", samples1[0], samples1[1], samples1[2], samples1[3]);
            sleep(TIME_MS2I(250));
        }
    }
} adcEchoThread;

/*
 * Application entry point.
 */
int main(void) {

    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    Shell::start(HIGHPRIO);

    /*
     * Activates the ADC1 driver and the temperature sensor.
     */
    palSetPadMode(GPIOC, GPIOC_ADC1_IN12, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, GPIOC_ADC1_IN13, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, GPIOC_ADC1_IN14, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, GPIOC_ADC1_IN15, PAL_MODE_INPUT_ANALOG);
    adcStart(&ADCD1, NULL);
    adcSTM32EnableTSVREFE();

    adcEchoThread.start(NORMALPRIO);

    Buzzer::play_sound(Buzzer::sound_startup, NORMALPRIO);

#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(1);
#endif
    return 0;
}
