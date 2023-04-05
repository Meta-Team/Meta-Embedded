//
// Created by Tianyi Han on 3/14/2023.
//

#ifndef META_EMBEDDED_BMI088_H
#define META_EMBEDDED_BMI088_H

#include "ch.hpp"
#include "hal.h"
#include "common_macro.h"
#include "ahrs_math.hpp"
#include "bmi088_reg.h"

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME_US 150

#define BMI088_ACCEL_READ_OFFSET 2
#define BMI088_GYRO_READ_OFFSET 1

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125


#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f


#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

#if defined(BOARD_RM_C)
#define BMI088_SPI_DRIVER SPID1
#define BMI088_SPI_CS_ACCEL_PAD GPIOA
#define BMI088_SPI_CS_ACCEL_PIN GPIOA_CS1_ACCEL
#define BMI088_SPI_CS_GYRO_PAD GPIOB
#define BMI088_SPI_CS_GYRO_PIN GPIOB_CS1_GYRO
#else
#error "BMI088 interface has not been defined for selected board"
#endif

/**
 * @name IMUInterface
 * @brief Read/write data from the BMI088 IMU through the SPI protocal
 * @usage 1. Call start() to enable driver and updating thread.
 * @note This interface is modified based on interface/ahrs/imu_on_board.h
 */
class BMI088Interface : protected chibios_rt::BaseStaticThread<512> {
public:
    /**
     * Initialize BMI088driver
     */
    chibios_rt::ThreadReference start(tprio_t prio) override;

    /**
     * Load external calibration data
     * @param gyro_bias_   Gyro bias value
     * @note To skip initial calibration at init(), call this function BEFORE init()
     */
    void load_calibration_data(Vector3D gyro_bias_);

    /**
     * Return whether the device is ready
     */
    bool ready() const { return imu_startup_calibrated; };

    Vector3D get_gyro() const { return gyro; }
    Vector3D get_accel() const { return accel; }
    float get_temp() const { return temperature; }

    /**
     * Update gyro, accel and magnet
     * @note Should be called from NORMAL state (not in locks)
     */
    void update();

    time_msecs_t imu_update_time = 0;   // last update time from system start [ms]

private:

    Vector3D gyro;    // data from gyroscope [deg/s]
    Vector3D accel;   // data from accelerometer [m/s^2]
    float temperature;

    Vector3D gyro_raw;   // raw (biased) data of gyro

    float gyro_psc;   // the coefficient converting the raw data to degree
    float accel_psc;  // the coefficient converting the raw data to m/s^2

    // TODO: calculate RX_BUF_SIZE
    static constexpr size_t RX_BUF_SIZE = 6 /* gyro */ + 2 /* temperature */ + 6 /* accel */ + 7 /* ist8310*/;
    uint8_t rx_buf[RX_BUF_SIZE];

    Vector3D gyro_bias;        // averaged gyro value when "static"
    Vector3D temp_gyro_bias;   // temp sum of gyro for calibration

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
     * @param data_offset Bytes needed to discard before the actual data, 2 for accel, 1 for gyro
     */
    void bmi088_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t data_len, uint8_t data_offset);

    friend class AHRSCalibrationThread;
};

#endif //META_EMBEDDED_BMI088_H
