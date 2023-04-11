//
// Created by Tianyi Han on 3/14/2023.
//

#ifndef META_EMBEDDED_IST8310_H
#define META_EMBEDDED_IST8310_H

#include "ch.hpp"
#include "hal.h"
#include "common_macro.h"

/**
 * @name IST8310Interface
 * @brief Read/write data from the IST8310 Magnetometer through the I2C bus
 * @usage 1. Call start() to enable driver and updating thread.
 * @note This interface is modified based on ChibiOS/testhal/STM32/STM32F1XX/I2C
 */
class IST8310Interface : protected chibios_rt::BaseStaticThread<512> {
public:
    /**
     * Initialize IST8310driver
     */
    chibios_rt::ThreadReference start(tprio_t prio) override;

    /**
     * Load external calibration data
     * @param mag_bias_   magnetometer bias value
     * @note To skip initial calibration at init(), call this function BEFORE init()
     */
    void load_calibration_data(Vector3D compass_bias_);

    /**
     * Return whether the device is ready
     */
    bool ready() const { return compass_startup_calibrated; };

    Vector3D get_compass() const { return compass; }

    float get_temp() const { return temperature; }

    /**
     * Update magnet
     * @note Should be called from NORMAL state (not in locks)
     */
    void update();

    time_msecs_t mag_update_time = 0;   // last update time from system start [ms]

private:

    Vector3D compass;    // data from magnetometer

    float temperature;

    Vector3D compass_raw;   // raw (biased) data

    float compass_psc;   // the coefficient converting the raw data

    // TODO: calculate RX_BUF_SIZE
    static constexpr size_t RX_BUF_SIZE = 6 /* gyro */ + 2 /* temperature */ + 6 /* accel */ + 7 /* ist8310*/;
    uint8_t rx_buf[RX_BUF_SIZE];

    Vector3D compass_bias;        // averaged gyro value when "static"
    Vector3D temp_compass_bias;   // temp sum of gyro for calibration

    const float TEMPERATURE_BIAS = 0.0f;

    // If changes in x, y, z of gyro is in this range BMI088 is regarded as "static"
    const float STATIC_RANGE = 0.5f;

    const unsigned BIAS_SAMPLE_COUNT = 500;
    unsigned static_measurement_count;
    // When static_measurement_count reaches BIAS_SAMPLE_COUNT, calibration is performed.

    bool imu_startup_calibrated = false;

    time_msecs_t last_calibration_time = 0;

    static constexpr int UPDATE_THREAD_INTERVAL = 1;  // [ms]

    /// BMI088 register set up error code
    enum {
        BMI088_NO_ERROR = 0x00,
        BMI088_ACC_PWR_CTRL_ERROR = 0x01,
        BMI088_ACC_PWR_CONF_ERROR = 0x02,
        BMI088_ACC_CONF_ERROR = 0x03,
        BMI088_ACC_SELF_TEST_ERROR = 0x04,
        BMI088_ACC_RANGE_ERROR = 0x05,
        BMI088_INT1_IO_CTRL_ERROR = 0x06,
        BMI088_INT_MAP_DATA_ERROR = 0x07,
        BMI088_GYRO_RANGE_ERROR = 0x08,
        BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
        BMI088_GYRO_LPM1_ERROR = 0x0A,
        BMI088_GYRO_CTRL_ERROR = 0x0B,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
        BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

        BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
        BMI088_SELF_TEST_GYRO_ERROR = 0x40,
        BMI088_NO_SENSOR = 0xFF,
    };

    /// BMI088 accel register configuration [reg, data, error_code]
    const uint8_t write_bmi088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
            {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
            {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
    };

    /// BMI088 gyro register configuration [reg, data, error_code]
    const uint8_t write_bmi088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] = {
            {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
            {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
            {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
            {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
            {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
            {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
    };

    /**
     * Initialize the BMI088 IMU
     */
    uint8_t init();

    /**
     * Initialize the accelerometer
     */
    bool init_accel();

    /**
     * Initialize the gyroscope
     */
    bool init_gyro();

    /**
     * Main function of the thread
     */
    void main() override;

    /**
     * Write to a single register in BMI088
     * @param reg Register address
     * @param data Data to write
     */
    void bmi088_write_reg(uint8_t reg, uint8_t data);

    /**
     * Read from one or multiple registers in BMI088
     * @param reg Register address
     * @param rx_data Data to read
     * @param data_len Data length
     * @param data_offset Bytes need to be discarded before the actual data, 2 for accel, 1 for gyro
     */
    void bmi088_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t data_len, uint8_t data_offset);

    friend class AHRSCalibrationThread;
};

#endif //META_EMBEDDED_IST8310_H
