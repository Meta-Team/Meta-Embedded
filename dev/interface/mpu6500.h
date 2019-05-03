#ifndef META_MPU6500_H_
#define META_MPU6500_H_

/**
 * This file contains interface and thread of MPU6500.
 */

#include "ch.hpp"
#include "hal.h"

#include "common_macro.h"

#include "mpu6500_reg.h"

#if defined(BOARD_RM_2017)
// SPI5
#define MPU6500_SPI_DRIVER SPID5
#define MPU6500_SPI_CS_PAD GPIOF
#define MPU6500_SPI_CS_PIN GPIOF_SPI5_NSS
#elif defined(BOARD_RM_2018_A)
// SPI5
#define MPU6500_SPI_DRIVER SPID5
#define MPU6500_SPI_CS_PAD GPIOF
#define MPU6500_SPI_CS_PIN GPIOF_SPI5_NSS
#else
#error "MPU6500 interface has not been defined for selected board"
#endif


typedef float matrix3[3][3];

class Vector3D {
public:

    float x;
    float y;
    float z;
    Vector3D():x(0),y(0),z(0) {};
    Vector3D(float a, float b, float c):x(a),y(b),z(c) {};
    Vector3D(float a[3]):x(a[0]), y(a[1]), z(a[2]) {};

    /**
    * @brief rotate the vector with the bias matrix
    * @param vector3D and bias matrix
    * @return a vector
    */
    friend const Vector3D operator*(Vector3D a, matrix3 b) {
        Vector3D converted;
        converted.x = b[0][0] * a.x + b[0][1] * a.y + b[0][2] * a.z;
        converted.y = b[1][0] * a.x + b[1][1] * a.y + b[1][2] * a.z;
        converted.z = b[2][0] * a.x + b[2][1] * a.y + b[2][2] * a.z;
        return converted;
    }

    const Vector3D crossMultiply(Vector3D b) {
        Vector3D ans;
        ans.x = y * b.z - z * b.y;
        ans.y = z * b.x - x * b.z;
        ans.z = x * b.y - y * b.x;
        return ans;
    }

};

/**
 * @name MPU6500Controller
 * @brief Interface to get MPU6500 data
 * @usage 1. Call start() to enable MPU6500 driver and the data fetching thread
 *        2. Fetch data from angle_speed, angle_speed, etc.
 */
class MPU6500 {

public:

    static Vector3D angle_speed;  // final data of gyro
    static Vector3D a_component;  // final data of acceleration
    static float temperature;

    static time_msecs_t last_update_time;

    static float dt;

    /**
     * Start MPU6500 driver and the thread of data fetching
     * @param prio
     * @return
     */
    static bool start(tprio_t prio);

private:

    /**
     * @brief read data from mpu6000 and convert to angel_speed_t type
     * @param none
     * @return an angel_speed_t type
     * @note this function is temporary for gyro. Later we should add acceleration to it.
     */
    static void getData();

    static float prev_t;

    static float _gyro_psc;  // get the coefficient converting the raw data to degree
    static float _accel_psc;  // get the coefficient converting the raw data to g

    static Vector3D _gyro_bias;  // for gyro bias
    static matrix3 _accel_bias;  // a matrix for accelerate bias

    static void mpu6500_write_reg(uint8_t reg_addr, uint8_t value);

    /**
     * MPU6500_ACCEL_CONFIG_2, [2:0] bits
     */
    typedef enum{
        MPU6500_ADLPF_460HZ =  0,
        MPU6500_ADLPF_184HZ =  1,
        MPU6500_ADLPF_92HZ  =  2,
        MPU6500_ADLPF_41HZ  =  3,
        MPU6500_ADLPF_20HZ  =  4,
        MPU6500_ADLPF_10HZ  =  5,
        MPU6500_ADLPF_5HZ   =  6
    } acc_dlpf_config_t;


    /**
     * Gyro Digital Low-Pass Filter Configuration
     * Register 26 – Configuration, [2:0] bits DLPF_CFG
     * @note to enable DLPF, [1:0] FCHOICE_B at Reg 27 should be set to 00 (default by reset)
     * @note when DLPF is enabled, the sample rate is 1kHz by default (can be changed with Reg 25 – Sample Rate Divider)
     */
    typedef enum{
        MPU6500_DLPF_250HZ  =  0,
        MPU6500_DLPF_184HZ  =  1,
        MPU6500_DLPF_92HZ   =  2,
        MPU6500_DLPF_41HZ   =  3,  // √
        MPU6500_DLPF_20HZ   =  4,
        MPU6500_DLPF_10HZ   =  5,
        MPU6500_DLPF_5HZ    =  6,
        MPU6500_DLPF_3600HZ =  7
    } dlpf_config_t;

    /**
     * Gyro full scale config
     * Register 27 - Gyroscope Configuration, [4:3] bits GYRO_FS_SEL
     */
    typedef enum {
        MPU6500_GYRO_SCALE_250 = 0,  // range of 250 dps with sensitivity factor 131 √
        MPU6500_GYRO_SCALE_500 = 1,  // range of 500 dps with sensitivity factor 65.5
        MPU6500_GYRO_SCALE_1000 = 2, // range of 1000 dps with sensitivity factor 32.8
        MPU6500_GYRO_SCALE_2000 = 3  // range of 1000 dps with sensitivity factor 16.4
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

    /*
     * type for mpu6500 basic config
     */
    typedef struct {
        gyro_scale_t _gyro_scale;
        accel_scale_t _accel_scale;
        dlpf_config_t _dlpf_config;
        acc_dlpf_config_t _acc_dlpf_config;
    } mpu6500_config_t;


    class MPU6500UpdateThread : public chibios_rt::BaseStaticThread<512> {
        void main() final {
            setName("mpu6500");
            while (!shouldTerminate()) {
                MPU6500::getData();
                sleep(TIME_MS2I(mpu6500_thread_update_interval));
            }
        }
    };

    static MPU6500UpdateThread updateThread;


private:

    /** Configurations **/

    static constexpr mpu6500_config_t config = {MPU6500_GYRO_SCALE_250,  // Gyro full scale 250 dps (degree per second)
                                                MPU6500_ACCEL_SCALE_8G,  // Accel full scale 8g
                                                MPU6500_DLPF_41HZ,       // Gyro digital low-pass filter 41Hz
                                                MPU6500_ADLPF_20HZ};     // Accel digital low-pass filter 20Hz

    static constexpr unsigned int mpu6500_thread_update_interval = 1;  // read interval 1ms (1kHz)
};

#endif