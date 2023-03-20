//
// Created by Tianyi Han on 3/14/2023.
//

#include "bmi088_interface.h"
#include "led.h"

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

// TODO: modify spi configuration refer to the sensor manual
static const SPIConfig SPI1_cfg = {
        /// Enables the circular buffer mode.
        false,
        /// Operation complete callback or @p NULL.
        nullptr,
        /// The chip select port.
        BMI088_SPI_CS_PAD,
        /// The chip select pad number.
        BMI088_SPI_CS_PIN,
        /// SPI CR1 register initialization data.
        SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_MSTR |
        SPI_CR1_CPHA | SPI_CR1_CPOL, //Set CPHA and CPOL to be 1
        /// SPI CR2 register initialization data.
        0
    };

#if defined(BMI088_USE_SPI)

/**
 * @note SPI Read/Write Protocol
 * 1. (ChibiOS) Lock the SPI driver (spiAcquireBus)
 * 2. CS (Chip Select) down (spiSelect)
 * 3. Send register address N (bit 7 is 0 for write, 1 for read)
 * 4. Read/Write register N
 * (Optional, repeated) Read/Write register N+1, N+2, N+3, ...
 * 5. CS up (spiUnselect)
 * 6. (ChibiOS) unlock the SPI driver (spiReleaseBus)
 */

/**
 * Write one or more register(s) through SPI
 * @param data [(register address N | write bit), data for register N, data for register N+1, ...]
 * @param len    Data array length
 */
void spi_read_write_data(const uint8_t *tx_data, size_t tx_len, uint8_t *rx_data, size_t rx_len) {
    spiAcquireBus(&BMI088_SPI_DRIVER);
    spiSelect(&BMI088_SPI_DRIVER);
    spiSend(&BMI088_SPI_DRIVER, tx_len, tx_data);
    spiReceive(&BMI088_SPI_DRIVER, rx_len, rx_data);
    spiUnselect(&BMI088_SPI_DRIVER);
    spiReleaseBus(&BMI088_SPI_DRIVER);
}

void BMI088Interface::bmi088_write_single_reg(uint8_t reg, uint8_t tx_data){
    uint8_t dummy_rx_data;
    spi_read_write_data(&reg, 1, &dummy_rx_data, 1);
    spi_read_write_data(&tx_data, 1, &dummy_rx_data, 1);
}

void BMI088Interface::bmi088_read_single_reg(uint8_t reg, uint8_t *rx_data){
    uint8_t dummy_rx_data;
    uint8_t tx_data = 0x55;
    reg |= 0x80;
    spi_read_write_data(&reg, 1, &dummy_rx_data, 1);
    spi_read_write_data(&tx_data, 1, rx_data, 1);
}

void BMI088Interface::bmi088_read_muli_reg(uint8_t reg, uint8_t *rx_data, uint8_t rx_len){
    uint8_t dummy_rx_data;
    uint8_t tx_data = 0x55;
    reg |= 0x80;
    spi_read_write_data(&reg, 1, &dummy_rx_data, 1);
    while (rx_len){
        spi_read_write_data(&tx_data, 1, rx_data, 1);
        rx_data++;
        rx_len--;
    }
}

void BMI088Interface::bmi088_accel_write_single_reg(uint8_t reg, uint8_t tx_data){
    palClearPad(GPIOA, GPIOA_CS1_ACCEL);
    bmi088_write_single_reg(reg, tx_data);
    palSetPad(GPIOA, GPIOA_CS1_ACCEL);
}

void BMI088Interface::bmi088_accel_read_single_reg(uint8_t reg, uint8_t *rx_data){
    palClearPad(GPIOA, GPIOA_CS1_ACCEL);
    uint8_t dummy_rx_data;
    uint8_t tx_data = 0x55;
    reg |= 0x80;
    spi_read_write_data(&reg, 1, &dummy_rx_data, 1);
    spi_read_write_data(&tx_data, 1, &dummy_rx_data, 1);
    spi_read_write_data(&tx_data, 1, rx_data, 1);
    palSetPad(GPIOA, GPIOA_CS1_ACCEL);
}

void BMI088Interface::bmi088_accel_read_muli_reg(uint8_t reg, uint8_t *rx_data, uint8_t rx_len){
    palClearPad(GPIOA, GPIOA_CS1_ACCEL);
    uint8_t dummy_rx_data;
    uint8_t tx_data = 0x55;
    reg |= 0x80;
    // TODO: reg | 0x80 is sent twice, could be a bug
    spi_read_write_data(&reg, 1, &dummy_rx_data, 1);
    bmi088_read_muli_reg(reg, rx_data, rx_len);
    palSetPad(GPIOA, GPIOA_CS1_ACCEL);
}

void BMI088Interface::bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data){
    palClearPad(GPIOB, GPIOB_CS1_GYRO);
    bmi088_write_single_reg(reg, tx_data);
    palSetPad(GPIOB, GPIOB_CS1_GYRO);
}

void BMI088Interface::bmi088_gyro_read_single_reg(uint8_t reg, uint8_t *rx_data){
    palClearPad(GPIOB, GPIOB_CS1_GYRO);
    uint8_t dummy_rx_data;
    uint8_t tx_data = 0x55;
    reg |= 0x80;
    spi_read_write_data(&reg, 1, &dummy_rx_data, 1);
    spi_read_write_data(&tx_data, 1, &dummy_rx_data, 1);
    spi_read_write_data(&tx_data, 1, rx_data, 1);
    palSetPad(GPIOB, GPIOB_CS1_GYRO);
}

void BMI088Interface::bmi088_gyro_read_muli_reg(uint8_t reg, uint8_t *rx_data, uint8_t rx_len){
    palClearPad(GPIOB, GPIOB_CS1_GYRO);
    uint8_t dummy_rx_data;
    uint8_t tx_data = 0x55;
    reg |= 0x80;
    // TODO: reg | 0x80 is sent twice, could be a bug
    spi_read_write_data(&reg, 1, &dummy_rx_data, 1);
    bmi088_read_muli_reg(reg, rx_data, rx_len);
    palSetPad(GPIOB, GPIOB_CS1_GYRO);
}

#elif defined(BMI088_USE_I2C)

#endif

chibios_rt::ThreadReference BMI088Interface::start(tprio_t prio) {
    init();
    return chibios_rt::BaseStaticThread<512>::start(prio);
}

uint8_t BMI088Interface::init() {
#if defined(BMI088_USE_SPI)
    /// Start SPI driver
    spiStart(&BMI088_SPI_DRIVER, &SPI1_cfg);

#elif defined(BMI088_USE_I2C)

#endif
    /// Initialize timer10 ch1 here, if heating is needed

    /// Initialize BMI088
    chThdSleepMilliseconds(100);  // wait for BMI088 to reset, see data sheet

    uint8_t error = BMI088_NO_ERROR;
    error |= init_accel();
    error |= init_gyro();

    return error;
}

bool BMI088Interface::init_accel() {
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    /// Check communication
    bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);
    bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

    /// Accel software reset
    bmi088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

    /// Check communication is normal after reset
    bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);
    bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

    /// Check chip ID
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    /// Set accel sensor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {

        bmi088_accel_write_single_reg(write_bmi088_accel_reg_data_error[write_reg_num][0], write_bmi088_accel_reg_data_error[write_reg_num][1]);
        chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

        bmi088_accel_read_single_reg(write_bmi088_accel_reg_data_error[write_reg_num][0], &res);
        chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

        if (res != write_bmi088_accel_reg_data_error[write_reg_num][1]) {
            return write_bmi088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

bool BMI088Interface::init_gyro() {
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    /// Check communication
    bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);
    bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

    /// Reset the gyro sensor
    bmi088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);
    //check communication is normal after reset
    bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);
    bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

    /// Check chip ID
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    /// Set gyro sensor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
        bmi088_gyro_write_single_reg(write_bmi088_gyro_reg_data_error[write_reg_num][0], write_bmi088_gyro_reg_data_error[write_reg_num][1]);
        chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

        bmi088_gyro_read_single_reg(write_bmi088_gyro_reg_data_error[write_reg_num][0], &res);
        chThdSleepMicroseconds(BMI088_COM_WAIT_SENSOR_TIME_US);

        if (res != write_bmi088_gyro_reg_data_error[write_reg_num][1]) {
            return write_bmi088_gyro_reg_data_error[write_reg_num][2];
        }
    }
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

        bmi088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
        bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
        accel.x = bmi088_raw_temp * BMI088_ACCEL_SEN;
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        accel.y = bmi088_raw_temp * BMI088_ACCEL_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        accel.z = bmi088_raw_temp * BMI088_ACCEL_SEN;

        bmi088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
        if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            gyro.x = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            gyro.y = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            gyro.z = bmi088_raw_temp * BMI088_GYRO_SEN;
        }

        bmi088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

        if (bmi088_raw_temp > 1023) {
            bmi088_raw_temp -= 2048;
        }

        temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

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