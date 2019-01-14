#ifndef META_MPU6500_H_
#define META_MPU6500_H_

#include "ch.hpp"
#include "hal.h"

typedef struct {
    float wx;
    float wy;
    float wz;
} angel_speed_t;

typedef struct {
    float ax;
    float ay;
    float az;
} accelerate_t;

/*typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quaternion_t;*/

class IMUController {

public:

    /**
     * Scale config for gyro
     * MPU6500_GYRO_CONFIG, [4:3] bits, shift when set SPI
     */
    typedef enum {
        MPU6500_GYRO_SCALE_250 = 0,  // range of 250 with sensitivity factor 131
        MPU6500_GYRO_SCALE_500 = 1, // range of 500 with sensitivity factor 65.5
        MPU6500_GYRO_SCALE_1000 = 2, // √ range of 1000 with sensitivity factor 32.8
        MPU6500_GYRO_SCALE_2000 = 3 // range of 1000 with sensitivity factor 16.4
    } gyro_scale_t;

    /**
     * Scale config for acceleration
     * MPU6500_ACCEL_CONFIG, [4:3] bits, shift when set SPI
     */
    typedef enum {
        MPU6500_ACCEL_SCALE_2G = 0,
        MPU6500_ACCEL_SCALE_4G = 1,
        MPU6500_ACCEL_SCALE_8G = 2, // √
        MPU6500_ACCEL_SCALE_16G = 3
    } accel_scale_t;

    /**
     * MPU6500_CONFIG, [0:2] bits
     */
    typedef enum{
        DLPF_250HZ  =  0,
        DLPF_184HZ  =  1,
        DLPF_92HZ   =  2,
        DLPF_41HZ   =  3,
        DLPF_20HZ   =  4,
        DLPF_10HZ   =  5,
        DLPF_5HZ    =  6,
        DLPF_3600HZ =  7
    } mpu6500_dlpf_config_t;

    typedef enum{
        ADLPF_460HZ =  0,
        ADLPF_184HZ =  1,
        ADLPF_92HZ  =  2,
        ADLPF_41HZ  =  3,
        ADLPF_20HZ  =  4,
        ADLPF_10HZ  =  5,
        ADLPF_5HZ   =  6
    } mpu6500_acc_dlpf_config_t;

    /**
    * @brief read data from mpu6000 and convert to angel_speed_t type
    * @param none
    * @return an angel_speed_t type
    * @note this function is temporary for gyro. Later we should add acceleration to it.
    */
    void getData();

    IMUController(gyro_scale_t input_gyro_config, accel_scale_t input_accel_config) {
/*        imu_q = {1, 0 ,0 ,0};*/
        _gyro_config = input_gyro_config;
        _accel_config = input_accel_config;
        dt = 0;
        prev_t = 0;
        IMUInit();
    }

    static void start(gyro_scale_t gyro_config, accel_scale_t accel_config);

private:

    static SPIDriver *spi_driver; // TODO: set spi

    float dt;
    float prev_t;


    float _gyro_bias;  // for gyro bias
    float _accel_bias[3][3];  // a matrix for accel bias (need the support of matrix)

/*    quaternion_t imu_q;*/

    angel_speed_t gyro_data;  // final data of gyro
    accelerate_t accel_data;  // final data of acceleration

    /**
    * @brief Initialize the IMU parameter
    * @param none
    * @return none
    */
    void IMUInit();

    static constexpr gyro_scale_t mpu6500_gyro_scale = MPU6500_GYRO_SCALE_1000;
    static constexpr accel_scale_t mpu6500_accel_scale = MPU6500_ACCEL_SCALE_8G;
};

/** MPU6500 Register Maps **/
#define MPU6500_SELF_TEST_XG        (0x00)
#define MPU6500_SELF_TEST_YG        (0x01)
#define MPU6500_SELF_TEST_ZG        (0x02)
#define MPU6500_SELF_TEST_XA        (0x0D)
#define MPU6500_SELF_TEST_YA        (0x0E)
#define MPU6500_SELF_TEST_ZA        (0x0F)
#define MPU6500_XG_OFFSET_H         (0x13)
#define MPU6500_XG_OFFSET_L         (0x14)
#define MPU6500_YG_OFFSET_H         (0x15)
#define MPU6500_YG_OFFSET_L         (0x16)
#define MPU6500_ZG_OFFSET_H         (0x17)
#define MPU6500_ZG_OFFSET_L         (0x18)
#define MPU6500_SMPLRT_DIV          (0x19)
#define MPU6500_CONFIG              (0x1A)
#define MPU6500_GYRO_CONFIG         (0x1B)
#define MPU6500_ACCEL_CONFIG        (0x1C)
#define MPU6500_ACCEL_CONFIG_2      (0x1D)
#define MPU6500_LP_ACCEL_ODR        (0x1E)
#define MPU6500_MOT_THR             (0x1F)
#define MPU6500_FIFO_EN             (0x23)
#define MPU6500_I2C_MST_CTRL        (0x24)
#define MPU6500_I2C_SLV0_ADDR       (0x25)
#define MPU6500_I2C_SLV0_REG        (0x26)
#define MPU6500_I2C_SLV0_CTRL       (0x27)
#define MPU6500_I2C_SLV1_ADDR       (0x28)
#define MPU6500_I2C_SLV1_REG        (0x29)
#define MPU6500_I2C_SLV1_CTRL       (0x2A)
#define MPU6500_I2C_SLV2_ADDR       (0x2B)
#define MPU6500_I2C_SLV2_REG        (0x2C)
#define MPU6500_I2C_SLV2_CTRL       (0x2D)
#define MPU6500_I2C_SLV3_ADDR       (0x2E)
#define MPU6500_I2C_SLV3_REG        (0x2F)
#define MPU6500_I2C_SLV3_CTRL       (0x30)
#define MPU6500_I2C_SLV4_ADDR       (0x31)
#define MPU6500_I2C_SLV4_REG        (0x32)
#define MPU6500_I2C_SLV4_DO         (0x33)
#define MPU6500_I2C_SLV4_CTRL       (0x34)
#define MPU6500_I2C_SLV4_DI         (0x35)
#define MPU6500_I2C_MST_STATUS      (0x36)
#define MPU6500_INT_PIN_CFG         (0x37)
#define MPU6500_INT_ENABLE          (0x38)
#define MPU6500_INT_STATUS          (0x3A)
#define MPU6500_ACCEL_XOUT_H        (0x3B)
#define MPU6500_ACCEL_XOUT_L        (0x3C)
#define MPU6500_ACCEL_YOUT_H        (0x3D)
#define MPU6500_ACCEL_YOUT_L        (0x3E)
#define MPU6500_ACCEL_ZOUT_H        (0x3F)
#define MPU6500_ACCEL_ZOUT_L        (0x40)
#define MPU6500_TEMP_OUT_H          (0x41)
#define MPU6500_TEMP_OUT_L          (0x42)
#define MPU6500_GYRO_XOUT_H         (0x43)
#define MPU6500_GYRO_XOUT_L         (0x44)
#define MPU6500_GYRO_YOUT_H         (0x45)
#define MPU6500_GYRO_YOUT_L         (0x46)
#define MPU6500_GYRO_ZOUT_H         (0x47)
#define MPU6500_GYRO_ZOUT_L         (0x48)
#define MPU6500_EXT_SENS_DATA_00    (0x49)
#define MPU6500_EXT_SENS_DATA_01    (0x4A)
#define MPU6500_EXT_SENS_DATA_02    (0x4B)
#define MPU6500_EXT_SENS_DATA_03    (0x4C)
#define MPU6500_EXT_SENS_DATA_04    (0x4D)
#define MPU6500_EXT_SENS_DATA_05    (0x4E)
#define MPU6500_EXT_SENS_DATA_06    (0x4F)
#define MPU6500_EXT_SENS_DATA_07    (0x50)
#define MPU6500_EXT_SENS_DATA_08    (0x51)
#define MPU6500_EXT_SENS_DATA_09    (0x52)
#define MPU6500_EXT_SENS_DATA_10    (0x53)
#define MPU6500_EXT_SENS_DATA_11    (0x54)
#define MPU6500_EXT_SENS_DATA_12    (0x55)
#define MPU6500_EXT_SENS_DATA_13    (0x56)
#define MPU6500_EXT_SENS_DATA_14    (0x57)
#define MPU6500_EXT_SENS_DATA_15    (0x58)
#define MPU6500_EXT_SENS_DATA_16    (0x59)
#define MPU6500_EXT_SENS_DATA_17    (0x5A)
#define MPU6500_EXT_SENS_DATA_18    (0x5B)
#define MPU6500_EXT_SENS_DATA_19    (0x5C)
#define MPU6500_EXT_SENS_DATA_20    (0x5D)
#define MPU6500_EXT_SENS_DATA_21    (0x5E)
#define MPU6500_EXT_SENS_DATA_22    (0x5F)
#define MPU6500_EXT_SENS_DATA_23    (0x60)
#define MPU6500_I2C_SLV0_DO         (0x63)
#define MPU6500_I2C_SLV1_DO         (0x64)
#define MPU6500_I2C_SLV2_DO         (0x65)
#define MPU6500_I2C_SLV3_DO         (0x66)
#define MPU6500_I2C_MST_DELAY_CTRL  (0x67)
#define MPU6500_SIGNAL_PATH_RESET   (0x68)
#define MPU6500_MOT_DETECT_CTRL     (0x69)
#define MPU6500_USER_CTRL           (0x6A)
#define MPU6500_PWR_MGMT_1          (0x6B)
#define MPU6500_PWR_MGMT_2          (0x6C)
#define MPU6500_FIFO_COUNTH         (0x72)
#define MPU6500_FIFO_COUNTL         (0x73)
#define MPU6500_FIFO_R_W            (0x74)
#define MPU6500_WHO_AM_I            (0x75)    // mpu6500 id = 0x71
#define MPU6500_XA_OFFSET_H         (0x77)
#define MPU6500_XA_OFFSET_L         (0x78)
#define MPU6500_YA_OFFSET_H         (0x7A)
#define MPU6500_YA_OFFSET_L         (0x7B)
#define MPU6500_ZA_OFFSET_H         (0x7D)
#define MPU6500_ZA_OFFSET_L         (0x7E)

/** MPU6500_PWR_MGMT_1 **/
#define MPU6500_RESET               (0x80) // bit 7
#define MPU6500_AUTO_SELECT_CLK     (0x01)


#endif