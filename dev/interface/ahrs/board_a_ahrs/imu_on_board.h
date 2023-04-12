#ifndef META_MPU6500_H_
#define META_MPU6500_H_

#include "ch.hpp"
#include "hal.h"

#include "common_macro.h"

#include "ahrs_abstract.h"
#include "ahrs_math.hpp"

#if defined(BOARD_RM_2018_A)
#define MPU6500_SPI_DRIVER SPID5
#define MPU6500_SPI_CS_PAD GPIOF
#define MPU6500_SPI_CS_PIN GPIOF_SPI5_NSS
#else
#error "MPU6500 interface has not been defined for selected board"
#endif

/**
 * @name IMUOnBoard
 * @brief Interface to get on-board MPU6500 data
 * @usage 1. Call start() to enable MPU6500 driver and updating thread. You can call load_calibration_data() BEFORE
 *           to skip initial calibration and use external calibration data
 *        2. Make use of data from MPU6500, accel, etc.
 */
class IMUOnBoard : protected chibios_rt::BaseStaticThread<512> {
public:

    /**
     * Initialize MPU6500 and IST8310 driver
     */
    chibios_rt::ThreadReference start(tprio_t prio) override;

    /**
     * Load external calibration data
     * @param gyro_bias_   Gyro bias value
     * @note To skip initial calibration at init(), call this function BEFORE init()
     */
    void load_calibration_data(Vector3D gyro_bias_);

    /**
     * Temperature data [C]
     */
    float temperature = 0;

    /**
     * Whether IMU is ready.
     * */
    bool ready() const { return imu_startup_calibrated; };

    Vector3D get_gyro() const { return gyro; }
    Vector3D get_accel() const { return accel; }
    Vector3D get_magnet() const { return magnet; }

    /**
     * Update gyro, accel and magnet
     * @note Should be called from NORMAL state (not in locks)
     */
    void update();

    time_msecs_t imu_update_time = 0;   // last update time from system start [ms]

private:

    Vector3D gyro;    // data from gyroscope [deg/s]
    Vector3D accel;   // data from accelerometer [m/s^2]
    Vector3D magnet;  // magnet data [uT]

    Vector3D gyro_raw;   // raw (biased) data of gyro

    float gyro_psc;   // the coefficient converting the raw data to degree
    float accel_psc;  // the coefficient converting the raw data to m/s^2

    static constexpr size_t RX_BUF_SIZE = 6 /* gyro */ + 2 /* temperature */ + 6 /* accel */ + 7 /* ist8310*/;
    uint8_t rx_buf[RX_BUF_SIZE];

    Vector3D gyro_bias;        // averaged gyro value when "static"
    Vector3D temp_gyro_bias;   // temp sum of gyro for calibration

    const float TEMPERATURE_BIAS = 0.0f;

    // If changes in x, y, z of gyro is in this range MPU6500 is regarded as "static"
    const float STATIC_RANGE = 0.5f;

    const unsigned BIAS_SAMPLE_COUNT = 500;
    unsigned static_measurement_count;
    // When static_measurement_count reaches BIAS_SAMPLE_COUNT, calibration is performed.

    bool imu_startup_calibrated = false;

    time_msecs_t last_calibration_time = 0;

    /**
     * MPU6500_ACCEL_CONFIG_2, [2:0] bits
     */
    typedef enum {
        MPU6500_ADLPF_460HZ = 0,
        MPU6500_ADLPF_184HZ = 1,
        MPU6500_ADLPF_92HZ = 2,
        MPU6500_ADLPF_41HZ = 3,
        MPU6500_ADLPF_20HZ = 4,
        MPU6500_ADLPF_10HZ = 5,
        MPU6500_ADLPF_5HZ = 6
    } acc_dlpf_config_t;

    /**
     * Gyro Digital Low-Pass Filter Configuration
     * Register 26 – Configuration, [2:0] bits DLPF_CFG
     * @note to enable DLPF, [1:0] FCHOICE_B at Reg 27 should be set to 00 (default by reset)
     * @note when DLPF is enabled, the sample rate is 1kHz by default (can be changed with Reg 25 – Sample Rate Divider)
     */
    typedef enum {
        MPU6500_DLPF_250HZ = 0,
        MPU6500_DLPF_184HZ = 1,
        MPU6500_DLPF_92HZ = 2,
        MPU6500_DLPF_41HZ = 3,  // √
        MPU6500_DLPF_20HZ = 4,
        MPU6500_DLPF_10HZ = 5,
        MPU6500_DLPF_5HZ = 6,
        MPU6500_DLPF_3600HZ = 7
    } dlpf_config_t;

    /**
     * Gyro full scale config
     * Register 27 - Gyroscope Configuration, [4:3] bits GYRO_FS_SEL
     */
    typedef enum {
        MPU6500_GYRO_SCALE_250 = 0,  // range of 250 dps with sensitivity factor 131
        MPU6500_GYRO_SCALE_500 = 1,  // range of 500 dps with sensitivity factor 65.5
        MPU6500_GYRO_SCALE_1000 = 2, // range of 1000 dps with sensitivity factor 32.8 √
        MPU6500_GYRO_SCALE_2000 = 3  // range of 2000 dps with sensitivity factor 16.4
    } gyro_scale_t;

    /**
     * Scale config for acceleration
     * MPU6500_ACCEL_CONFIG, [4:3] bits, shift when set SPI
     */
    typedef enum {
        MPU6500_ACCEL_SCALE_2G = 0,
        MPU6500_ACCEL_SCALE_4G = 1,
        MPU6500_ACCEL_SCALE_8G = 2,  // √
        MPU6500_ACCEL_SCALE_16G = 3
    } accel_scale_t;

    typedef struct {
        gyro_scale_t _gyro_scale;
        accel_scale_t _accel_scale;
        dlpf_config_t _dlpf_config;
        acc_dlpf_config_t _acc_dlpf_config;
    } mpu6500_config_t;

    static constexpr int MPU6500_STARTUP_TIME = 5000; // [ms]

    static constexpr mpu6500_config_t CONFIG = {
            MPU6500_GYRO_SCALE_1000,  // Gyro full scale 1000 dps (degree per second)
            MPU6500_ACCEL_SCALE_8G,  // Accel full scale 8g
            MPU6500_DLPF_41HZ,       // Gyro digital low-pass filter 41Hz
            MPU6500_ADLPF_20HZ};  // Accel digital low-pass filter 20Hz

    void init_mpu6500();

    void init_ist8310();

    void main() override;

    static constexpr int UPDATE_THREAD_INTERVAL = 1;  // [ms]

    friend class AHRSCalibrationThread;
};

#endif