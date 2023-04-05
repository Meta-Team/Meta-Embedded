//
// Created by Tianyi Han on 3/14/2023.
//

#include "bmi088_interface.h"
#include "led.h"
#include "shell.h"

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

static const SPIConfig SPI1_accel_cfg = {
    false, // Enables the circular buffer mode.
    nullptr, // Operation complete callback or @p NULL.
    BMI088_SPI_CS_ACCEL_PAD, // The chip select port.
    BMI088_SPI_CS_ACCEL_PIN, // The chip select pad number.
    SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_MSTR | SPI_CR1_CPHA | SPI_CR1_CPOL, // SPI CR1 register initialization data. //Set CPHA and CPOL to be 1
    0 // SPI CR2 register initialization data.
};

static const SPIConfig SPI1_gyro_cfg = {
    false, // Enables the circular buffer mode.
    nullptr, // Operation complete callback or @p NULL.
    BMI088_SPI_CS_GYRO_PAD, // The chip select port.
    BMI088_SPI_CS_GYRO_PIN, // The chip select pad number.
    SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_MSTR | SPI_CR1_CPHA | SPI_CR1_CPOL, // SPI CR1 register initialization data. //Set CPHA and CPOL to be 1
    0 // SPI CR2 register initialization data.
};

/**
 * @note SPI Read/Write Protocol
 * 1. (ChibiOS) Lock the SPI driver (spiAcquireBus)
 * 2. CS (Chip Select) down (spiSelect)
 * 3. Read/write uint8_t data (tx_len = rx_len)
 * 4. CS up (spiUnselect)
 * 5. (ChibiOS) unlock the SPI driver (spiReleaseBus)
 */

/**
 * Write one or more register(s) through SPI
 * @param data [(register address N | write bit), data for register N, data for register N+1, ...]
 * @param len    Data array length
 */
void spi_read_write_data(const uint8_t *tx_data, uint8_t *rx_data, size_t data_len) {
    spiAcquireBus(&BMI088_SPI_DRIVER);
    spiSelect(&BMI088_SPI_DRIVER);
    spiExchange(&BMI088_SPI_DRIVER, data_len, tx_data, rx_data);
    spiUnselect(&BMI088_SPI_DRIVER);
    spiReleaseBus(&BMI088_SPI_DRIVER);
/*    for (int i=0; i<data_len; i++) {
        Shell::printf("tx: %x rx: %x" ENDL, *(tx_data+i), *(rx_data+i));
    }
    Shell::printf(ENDL);*/
}

void BMI088Interface::bmi088_write_reg(uint8_t reg, uint8_t data){
    uint8_t tx_data[2] = {reg, data};
    uint8_t dummy_rx_data[2];
    spi_read_write_data(tx_data, dummy_rx_data, 2);
}

void BMI088Interface::bmi088_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t data_len, uint8_t data_offset){
    uint8_t tx_data[data_len+data_offset];
    tx_data[0] = 0x80 | reg;
    uint8_t dummy_rx_data[data_len+data_offset];
    spi_read_write_data(tx_data, dummy_rx_data, data_len + data_offset);
    std::copy(dummy_rx_data + data_offset, dummy_rx_data + data_len + data_offset, rx_data);
}

chibios_rt::ThreadReference BMI088Interface::start(tprio_t prio) {
    uint8_t error = BMI088_NO_ERROR;
    error = init();
    Shell::printf("error: %d" ENDL, error);
    return chibios_rt::BaseStaticThread<512>::start(prio);
}

uint8_t BMI088Interface::init() {
    // Initialize timer10 ch1 here, if heating is needed

    // Initialize BMI088
    uint8_t error = BMI088_NO_ERROR;
    error |= init_accel();
    error |= init_gyro();

    return error;
}

bool BMI088Interface::init_accel() {
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    // Start accelerometer communication
    spiStart(&BMI088_SPI_DRIVER, &SPI1_accel_cfg);

    // Check communication
    bmi088_read_reg(BMI088_ACC_CHIP_ID, &res, 1, BMI088_ACCEL_READ_OFFSET);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);
    bmi088_read_reg(BMI088_ACC_CHIP_ID, &res, 1, BMI088_ACCEL_READ_OFFSET);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

    // Accel software reset
    bmi088_write_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    chThdSleepMilliseconds(5);

    // Check communication is normal after reset
    bmi088_read_reg(BMI088_ACC_CHIP_ID, &res, 1, BMI088_ACCEL_READ_OFFSET);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);
    bmi088_read_reg(BMI088_ACC_CHIP_ID, &res, 1, BMI088_ACCEL_READ_OFFSET);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

    /// Check chip ID
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    /// Set accel sensor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {

        bmi088_write_reg(write_bmi088_accel_reg_data_error[write_reg_num][0], write_bmi088_accel_reg_data_error[write_reg_num][1]);
        chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

        bmi088_read_reg(write_bmi088_accel_reg_data_error[write_reg_num][0], &res, 1, BMI088_ACCEL_READ_OFFSET);
        chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

        if (res != write_bmi088_accel_reg_data_error[write_reg_num][1]) {
            LED::blue_on();
            chThdSleepMilliseconds(1000);
            return write_bmi088_accel_reg_data_error[write_reg_num][2];
        }
    }

    // Stop accelerometer communication
    spiStop(&BMI088_SPI_DRIVER);
    return BMI088_NO_ERROR;
}

bool BMI088Interface::init_gyro() {
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    // Start gyroscope communication
    spiStart(&BMI088_SPI_DRIVER, &SPI1_gyro_cfg);

    /// Check communication
    bmi088_read_reg(BMI088_GYRO_CHIP_ID, &res, 1, BMI088_GYRO_READ_OFFSET);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);
    bmi088_read_reg(BMI088_GYRO_CHIP_ID, &res, 1, BMI088_GYRO_READ_OFFSET);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

    /// Reset the gyro sensor
    bmi088_write_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    chThdSleepMilliseconds(5);

    //check communication is normal after reset
    bmi088_read_reg(BMI088_GYRO_CHIP_ID, &res, 1, BMI088_GYRO_READ_OFFSET);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);
    bmi088_read_reg(BMI088_GYRO_CHIP_ID, &res, 1, BMI088_GYRO_READ_OFFSET);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

    /// Check chip ID
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    /// Set gyro sensor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
        bmi088_write_reg(write_bmi088_gyro_reg_data_error[write_reg_num][0], write_bmi088_gyro_reg_data_error[write_reg_num][1]);
        chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

        bmi088_read_reg(write_bmi088_gyro_reg_data_error[write_reg_num][0], &res, 1, BMI088_GYRO_READ_OFFSET);
        chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

        if (res != write_bmi088_gyro_reg_data_error[write_reg_num][1]) {
            return write_bmi088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    // Stop gyroscope communication
    spiStop(&BMI088_SPI_DRIVER);
    return BMI088_NO_ERROR;
}

void BMI088Interface::load_calibration_data(Vector3D gyro_bias_) {
    gyro_bias = gyro_bias_;
    last_calibration_time = SYSTIME;
}

void BMI088Interface::update() {
    /// Fetch data from SPI

    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    {
        uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
        int16_t bmi088_raw_temp;

        // Start accelerometer communication
        spiStart(&BMI088_SPI_DRIVER, &SPI1_accel_cfg);

        bmi088_read_reg(BMI088_ACCEL_XOUT_L, buf, 6, BMI088_ACCEL_READ_OFFSET);
        bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
        accel.x = bmi088_raw_temp * BMI088_ACCEL_SEN;
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        accel.y = bmi088_raw_temp * BMI088_ACCEL_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        accel.z = bmi088_raw_temp * BMI088_ACCEL_SEN;

        bmi088_read_reg(BMI088_TEMP_M, buf, 2, BMI088_ACCEL_READ_OFFSET);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

        if (bmi088_raw_temp > 1023) bmi088_raw_temp -= 2048;
        temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        // Stop accelerometer communication
        spiStop(&BMI088_SPI_DRIVER);

        // Start gyroscope communication
        spiStart(&BMI088_SPI_DRIVER, &SPI1_gyro_cfg);

        bmi088_read_reg(BMI088_GYRO_CHIP_ID, buf, 8, BMI088_GYRO_READ_OFFSET);
        if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            gyro.x = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            gyro.y = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            gyro.z = bmi088_raw_temp * BMI088_GYRO_SEN;
        }

        // Stop gyroscope communication
        spiStop(&BMI088_SPI_DRIVER);

        /// Decode data

        /// Gyro Calibration sampling

        /// Bias data
//        gyro_raw = new_gyro_raw;
//        gyro = gyro_raw + gyro_bias;
//        accel = accel_raw;

        /// Update info
//        imu_update_time = SYSTIME;

        /// Perform gyro re-bias
//        if (!imu_startup_calibrated && (static_measurement_count >= BIAS_SAMPLE_COUNT) && (SYSTIME > BMI088_STARTUP_TIME)) {
//
//            imu_startup_calibrated = true;
//            gyro_bias = temp_gyro_bias / static_measurement_count;
//            static_measurement_count = 0;
//            temp_gyro_bias = Vector3D(0, 0, 0);
//            last_calibration_time = SYSTIME;
//        }

    }
    chSysUnlock();  /// --- EXIT S-Locked state ---
}

void BMI088Interface::main() {
    setName("BMI088Interface");
    while (!shouldTerminate()) {
        update();
        sleep(TIME_MS2I(UPDATE_THREAD_INTERVAL));
    }
}