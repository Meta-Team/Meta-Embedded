//
// Created by Tianyi Han on 3/14/2023.
//

/**
 * This is device realize "read through write" paradigm. This is not
 * standard, but most of I2C devices use this paradigm.
 * You must write to device reading address, send restart to bus,
 * and then begin reading process.
 */

#include "ist8310_interface.h"
#include "led.h"
#include "shell.h"

/* device I2C address */
#define addr 0b0011101
#define IST8310_IIC_ADDRESS 0x0E

/* enable single byte read checks. Note: it does not work on STM32F1x */
#define TEST_SINGLE_BYTE_READ

/* autoincrement bit position. This bit needs to perform reading of
 * multiple bytes at one request */
#define AUTO_INCREMENT_BIT (1<<7)

/* slave specific addresses */
#define ACCEL_STATUS_REG  0x27
#define ACCEL_CTRL_REG1   0x20
#define ACCEL_OUT_DATA    0x28

/* buffers */
static uint8_t accel_rx_data[8];
static uint8_t accel_tx_data[8];

/*
 * I2C1 config.
 */
static const I2CConfig i2cfg1 = {
        OPMODE_I2C,
        400000,
        FAST_DUTY_CYCLE_2,
};

void IST8310Interface::ist8310_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t data_len, uint8_t data_offset){

}

void IST8310Interface::ist8310_write_reg(uint8_t reg, uint8_t data){

}

chibios_rt::ThreadReference IST8310Interface::start(tprio_t prio) {
    uint8_t error = IST8310_NO_ERROR;
    error = init();
    Shell::printf("error: %d" ENDL, error);
    return chibios_rt::BaseStaticThread<512>::start(prio);
}

uint8_t IST8310Interface::init() {
    // Initialize I2C communication
    i2cStart(&I2CD1, &i2cfg1);

    // Initialize IST8310
    uint8_t error = IST8310_NO_ERROR;

    /// testing code
    msg_t status = MSG_OK;
    sysinterval_t tmo = TIME_MS2I(4);

    /* configure accelerometer */
    accel_tx_data[0] = ACCEL_CTRL_REG1 | AUTO_INCREMENT_BIT;
    accel_tx_data[1] = 0b11100111;
    accel_tx_data[2] = 0b01000001;
    accel_tx_data[3] = 0b00000000;

    /* sending */
    i2cAcquireBus(&I2CD1);
    status = i2cMasterTransmitTimeout(&I2CD1, IST8310_IIC_ADDRESS,
                                      accel_tx_data, 4, NULL, 0, tmo);
    i2cReleaseBus(&I2CD1);

    osalDbgCheck(MSG_OK == status);

    // TODO: initialization code

    return error;
}

void IST8310Interface::load_calibration_data(Vector3D compass_bias_) {
    compass_bias = compass_bias_;
    last_calibration_time = SYSTIME;
}

void IST8310Interface::update() {

}

void IST8310Interface::main() {
    setName("IST8310Interface");
    while (!shouldTerminate()) {
        update();
        sleep(TIME_MS2I(UPDATE_THREAD_INTERVAL));
    }
}

/*
void lis3Start(void){
    msg_t status = MSG_OK;
    sysinterval_t tmo = TIME_MS2I(4);

    */
/* configure accelerometer *//*

    accel_tx_data[0] = ACCEL_CTRL_REG1 | AUTO_INCREMENT_BIT;
    accel_tx_data[1] = 0b11100111;
    accel_tx_data[2] = 0b01000001;
    accel_tx_data[3] = 0b00000000;

    */
/* sending *//*

    i2cAcquireBus(&I2CD1);
    status = i2cMasterTransmitTimeout(&I2CD1, addr,
                                      accel_tx_data, 4, NULL, 0, tmo);
    i2cReleaseBus(&I2CD1);

    osalDbgCheck(MSG_OK == status);
}

static void raw2g(uint8_t *raw, float *g) {
    int16_t tmp;

    for (size_t i=0; i<3; i++){
        tmp = raw[i*2] | (raw[i*2+1] << 8);
        g[i] = (float)tmp / 16384.0; */
/* convert raw value to G *//*

    }
}

void lis3GetAcc(float *result) {
    msg_t status = MSG_OK;
    sysinterval_t tmo = TIME_MS2I(4);

    */
/* read in burst mode *//*

    memset(accel_rx_data, 0x55, sizeof(accel_rx_data));
    accel_tx_data[0] = ACCEL_OUT_DATA | AUTO_INCREMENT_BIT;
    i2cAcquireBus(&I2CD1);
    status = i2cMasterTransmitTimeout(&I2CD1, addr,
                                      accel_tx_data, 1, accel_rx_data, 6, tmo);
    i2cReleaseBus(&I2CD1);
    osalDbgCheck(MSG_OK == status);
    raw2g(accel_rx_data, result);

#if defined(TEST_SINGLE_BYTE_READ)
    float accel_single_byte_check[3];
    const float check_threshold = 0.1;

    */
/* read data byte at a time *//*

    memset(accel_rx_data, 0x55, sizeof(accel_rx_data));
    accel_tx_data[0] = ACCEL_OUT_DATA;
    i2cAcquireBus(&I2CD1);
    for (size_t i=0; i<6; i++) {
    status = i2cMasterTransmitTimeout(&I2CD1, addr,
                            accel_tx_data, 1, &accel_rx_data[i], 1, tmo);
    osalDbgCheck(MSG_OK == status);
    accel_tx_data[0]++;
    }
    i2cReleaseBus(&I2CD1);
    raw2g(accel_rx_data, accel_single_byte_check);

    */
/* check results *//*

    for (size_t i=0; i<3; i++) {
    osalDbgCheck(fabsf(result[i] - accel_single_byte_check[i]) < check_threshold);
    }
#endif */
/* TEST_SINGLE_BYTE_READ *//*

}*/
