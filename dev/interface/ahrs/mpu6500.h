#ifndef META_MPU6500_H_
#define META_MPU6500_H_

#include "ch.hpp"
#include "hal.h"

#include "common_macro.h"

#include "ahrs_abstract.h"
#include "ahrs_math.hpp"

#include "mpu6500_reg.h"

#if defined(BOARD_RM_2018_A)
#define MPU6500_SPI_DRIVER SPID5
#define MPU6500_SPI_CS_PAD GPIOF
#define MPU6500_SPI_CS_PIN GPIOF_SPI5_NSS
#else
#error "MPU6500 interface has not been defined for selected board"
#endif

// Enable rotation matrix to transform accel into Z direction. AHRS requires accel WITHOUT rotation.
#define MPU6500_ENABLE_ACCEL_BIAS  FALSE

/**
 * @name MPUOnBoard
 * @brief Interface to get on-board MPU6500 data
 * @usage 1. Call start() to enable MPU6500 driver and updating thread. You can call load_calibration_data() BEFORE
 *           to skip initial calibration and use external calibration data
 *        2. Make use of data from MPU6500, accel, etc.
 */
class MPUOnBoard : virtual public AbstractMPU {

public:

    /* (From AbstractMPU)

    (public)
    Vector3D get_gyro();  // get data from gyroscope [deg/s]
    Vector3D get_accel();  // get data from accelerometer [m/s^2]
    time_msecs_t get_mpu_update_time();  // get last update time from system start [ms]

    (protected)
    Vector3D gyro;   // Data from gyroscope [deg/s]
    Vector3D accel;  // Data from accelerometer [m/s^2]
    time_msecs_t mpu_update_time = 0;  // Last update time from system start [ms]

    */

    /**
     * Start MPU6500 driver and the thread of data fetching
     * @param prio  Thread priority (recommended to be high enough)
     */
    void start(tprio_t prio);

    /**
     * Load external calibration data
     * @param gyro_bias_   Gyro bias value
     * @note To skip initial calibration at start(), call this function BEFORE start()
     */
    void load_calibration_data(Vector3D gyro_bias_);

    /**
     * Temperature data [C]
     */
    float temperature;


    MPUOnBoard() : updateThread(*this) {};

private:

    Vector3D gyro_orig;   // raw (biased) data of gyro
    Vector3D accel_orig;  // raw (not rotated) data from accel

    float gyro_psc;   // the coefficient converting the raw data to degree
    float accel_psc;  // the coefficient converting the raw data to m/s^2

    static constexpr size_t RX_BUF_SIZE = 0x0E;
    uint8_t rx_buf[RX_BUF_SIZE];


    Vector3D gyro_bias;        // averaged gyro value when "static"
    Vector3D temp_gyro_bias;   // temp sum of gyro for calibration
#if MPU6500_ENABLE_ACCEL_BIAS
    Matrix33 accel_rotation;   // a matrix to rotate accel
    Vector3D temp_accel_bias;  // temp sum of accel for calibration
#endif
    const float TEMPERATURE_BIAS = 0.0f;

    // If changes in x, y, z of gyro is in this range MPU6500 is regarded as "static"
    const float STATIC_RANGE = 0.2f;

    const unsigned BIAS_SAMPLE_COUNT = 500;
    // When static_measurement_count reaches BIAS_SAMPLE_COUNT, calibration is performed.
    unsigned static_measurement_count;

    time_msecs_t last_calibration_time = 0;

    void mpu6500_write_reg(uint8_t reg_addr, uint8_t value);  // helper function to use SPI
    void update();

    class UpdateThread : public chibios_rt::BaseStaticThread<512> {
    public:
        UpdateThread(MPUOnBoard& mpu_) : mpu(mpu_) {}

    private:
        static constexpr unsigned int THREAD_UPDATE_INTERVAL = 1;  // read interval 1ms (1kHz)
        MPUOnBoard& mpu;
        void main() final;
    } updateThread;


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
        MPU6500_GYRO_SCALE_2000 = 3  // range of 1000 dps with sensitivity factor 16.4
    } gyro_scale_t;

    /**
     * Scale config for acceleration
     * MPU6500_ACCEL_CONFIG, [4:3] bits, shift when set SPI
     */
    typedef enum {
        MPU6500_ACCEL_SCALE_2G = 0, // √
        MPU6500_ACCEL_SCALE_4G = 1,
        MPU6500_ACCEL_SCALE_8G = 2,
        MPU6500_ACCEL_SCALE_16G = 3
    } accel_scale_t;

    typedef struct {
        gyro_scale_t _gyro_scale;
        accel_scale_t _accel_scale;
        dlpf_config_t _dlpf_config;
        acc_dlpf_config_t _acc_dlpf_config;
    } mpu6500_config_t;

    static constexpr mpu6500_config_t CONFIG = {MPU6500_GYRO_SCALE_1000,  // Gyro full scale 1000 dps (degree per second)
                                     MPU6500_ACCEL_SCALE_2G,  // Accel full scale 8g
                                     MPU6500_DLPF_41HZ,       // Gyro digital low-pass filter 41Hz
                                     MPU6500_ADLPF_20HZ};     // Accel digital low-pass filter 20Hz

    friend void cmd_echo_gyro_bias(BaseSequentialStream *chp, int argc, char *argv[]);
    friend class AHRSCalibrationThread;
};

#endif