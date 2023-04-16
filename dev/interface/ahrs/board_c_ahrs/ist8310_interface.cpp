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

#define IST8310_IIC_ADDRESS 0x0E // device I2C address

// I2C1 config
static const I2CConfig i2cfg1 = {
        OPMODE_I2C,
        400000,
        FAST_DUTY_CYCLE_2,
};

void IST8310Interface::ist8310_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t rx_len) {
    tx_buf[0] = reg;
    i2cAcquireBus(&IST8310_SPI_DRIVER);
    i2cMasterTransmitTimeout(&IST8310_SPI_DRIVER, IST8310_IIC_ADDRESS,
                             tx_buf, 1, rx_data, rx_len, TIME_MS2I(4));
    i2cReleaseBus(&IST8310_SPI_DRIVER);
}

void IST8310Interface::ist8310_write_reg(uint8_t reg, uint8_t data) {
    tx_buf[0] = reg;
    tx_buf[1] = data;
    i2cAcquireBus(&IST8310_SPI_DRIVER);
    i2cMasterTransmitTimeout(&IST8310_SPI_DRIVER, IST8310_IIC_ADDRESS,
                             tx_buf, 2, NULL, 0, TIME_MS2I(4));
    i2cReleaseBus(&IST8310_SPI_DRIVER);
}

void IST8310Interface::ist8310_hard_reset() {
    palClearPad(GPIOG, GPIOG_RSTN_IST8310);
    chThdSleepMilliseconds(50);
    palSetPad(GPIOG, GPIOG_RSTN_IST8310);
    chThdSleepMilliseconds(50);
}

chibios_rt::ThreadReference IST8310Interface::start(tprio_t prio) {
    uint8_t error = IST8310_NO_ERROR;
    error = init();
    Shell::printf("error: %d" ENDL, error);
    return chibios_rt::BaseStaticThread<512>::start(prio);
}

uint8_t IST8310Interface::init() {
    // Initialize I2C communication
    i2cStart(&IST8310_SPI_DRIVER, &i2cfg1);

    // Perform hard reset
    ist8310_hard_reset();

    // Initialize IST8310
    uint8_t error = IST8310_NO_ERROR;
    uint8_t write_reg_num = 0;

    // Who am I
    ist8310_read_reg(IST8310_WHO_AM_I, rx_buf, 1);
    if (rx_buf[0] != IST8310_WHO_AM_I_VALUE) {
        return IST8310_NO_SENSOR;
    }

    //set ist8310 sensor config and check
    for (write_reg_num = 0; write_reg_num < IST8310_WRITE_REG_NUM; write_reg_num++) {
        ist8310_write_reg(ist8310_write_reg_data_error[write_reg_num][0], ist8310_write_reg_data_error[write_reg_num][1]);
        chThdSleepMicroseconds(150);
        ist8310_read_reg(ist8310_write_reg_data_error[write_reg_num][0], rx_buf, 1);
        chThdSleepMicroseconds(150);
        if (rx_buf[0] != ist8310_write_reg_data_error[write_reg_num][1])
        {
            return ist8310_write_reg_data_error[write_reg_num][2];
        }
    }

    return error;
}

void IST8310Interface::update() {
    ist8310_read_reg(IST8310_DATAXL, rx_buf, 6);

    raw_temp = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
    compass.x = MAG_SEN * raw_temp;
    raw_temp = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    compass.y = MAG_SEN * raw_temp;
    raw_temp = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
    compass.z = MAG_SEN * raw_temp;
}

void IST8310Interface::main() {
    setName("IST8310Interface");
    while (!shouldTerminate()) {
        update();
        sleep(TIME_MS2I(UPDATE_THREAD_INTERVAL));
    }
}
