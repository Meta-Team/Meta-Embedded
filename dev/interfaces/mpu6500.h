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

    /*
    * Scale config for gyro
    */
    typedef enum {
        MPU6500_GYRO_SCALE_250 = 0,  // range of 250 with sensitivity factor 131
        MPU6500_GYRO_SCALE_500 = 1, // range of 500 with sensitivity factor 65.5
        MPU6500_GYRO_SCALE_1000 = 2, // range of 1000 with sensitivity factor 32.8
        MPU6500_GYRO_SCALE_2000 = 3 // range of 1000 with sensitivity factor 16.4
    } gyro_scale;

    /*
    * Scale config for acceleration
    */
    typedef enum {
        MPU6500_ACCEL_SCALE_2G = 0,
        MPU6500_ACCEL_SCALE_4G = 1,
        MPU6500_ACCEL_SCALE_8G = 2,
        MPU6500_ACCEL_SCALE_16G = 3
    } accel_scale;

    /**
    * @brief read data from mpu6000 and convert to angel_speed_t type
    * @param none
    * @return an angel_speed_t type
    * @note this function is temporary for gyro. Later we should add acceleration to it.
    */
    void getData();

    IMUController(gyro_scale input_gyro_config, accel_scale input_accel_config) {
/*        imu_q = {1, 0 ,0 ,0};*/
        _gyro_config = input_gyro_config;
        _accel_config = input_accel_config;
        dt = 0;
        prev_t = 0;
        IMUInit();
    }

private:

    float dt;
    float prev_t;

    gyro_scale _gyro_config;
    accel_scale _accel_config;

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
};

#endif