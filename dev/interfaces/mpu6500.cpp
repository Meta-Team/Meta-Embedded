#include "mpu6500.h"
#include <cmath>

#define PI 3.14159265358979323846f
#define GRAV 9.80665f
#define MPU6500_RX_BUF_SIZE 0x0E
#define MPU6500_TX_BUF_SIZE 0x05

static uint8_t imuRXData[MPU6500_RX_BUF_SIZE];
static uint8_t imuTXData[MPU6500_TX_BUF_SIZE];

void IMUController::IMUInit() {
    // read to set bias here
    getData();
}

void IMUController::getData() {
    uint32_t current_time =  chVTGetSystemTimeX();
    this->dt = TIME_I2S(current_time - this->prev_t);
    this->prev_t = current_time;
    // get dt

    // get the cofficient converting the raw data to degree
    float _gyro_psc;
    switch(this->_gyro_config)
    {
        case MPU6500_GYRO_SCALE_250:
            _gyro_psc = (1.0f / 131.0f) * PI/180.0f;
            break;
        case MPU6500_GYRO_SCALE_500:
            _gyro_psc = (1.0f /  65.5f) * PI/180.0f;
            break;
        case MPU6500_GYRO_SCALE_1000:
            _gyro_psc = (1.0f /  32.8f) * PI/180.0f;
            break;
        case MPU6500_GYRO_SCALE_2000:
            _gyro_psc = (1.0f /  16.4f) * PI/180.0f;
            break;
    }

    float _accel_psc;
    switch(this->_accel_config)
    {
        case MPU6500_ACCEL_SCALE_2G:
            _accel_psc = (GRAV / 16384.0f);
            break;
        case MPU6500_ACCEL_SCALE_4G:
            _accel_psc = (GRAV /  8192.0f);
            break;
        case MPU6500_ACCEL_SCALE_8G:
            _accel_psc = (GRAV /  4096.0f);
            break;
        case MPU6500_ACCEL_SCALE_16G:
            _accel_psc = (GRAV /  2048.0f);
            break;
    }
    /*Here to read data*/

    float accel_x = _accel_psc * (int16_t)((imuRXData[ 0]<<8) | imuRXData[ 1]); // Accel X 
    float accel_y = _accel_psc * (int16_t)((imuRXData[ 2]<<8) | imuRXData[ 3]); // Accel Y 
    float accel_z = _accel_psc * (int16_t)((imuRXData[ 4]<<8) | imuRXData[ 5]); // Accel Z
    float gyro_x = _gyro_psc * (int16_t)((imuRXData[ 8]<<8) | imuRXData[ 9]);  // Gyro X 
    float gyro_y = _gyro_psc * (int16_t)((imuRXData[10]<<8) | imuRXData[11]);  // Gyro Y 
    float gyro_z = _gyro_psc * (int16_t)((imuRXData[12]<<8) | imuRXData[13]);  // Gyro Z 

    this->gyro_data.x = (gyro_x + _gyro_bias) / this->dt;
    this->gyro_data.y = (gyro_y + _gyro_bias) / this->dt;
    this->gyro_data.z = (gyro_z + _gyro_bias) / this->dt;

    Matrix accel_data_matrix(1, 3);
    // fill the matrix with accel_x, accel_y and accel_z
    this->accel_data = matrix_multiply_to_accelerate_t(accel_data_matrix, _accel_bias);
}