#include "mpu6500.h"
#include <cmath>

#define PI 3.14159265358979323846f
#define GRAV 9.80665f
#define MPU6500_RX_BUF_SIZE 0x0E
#define MPU6500_TX_BUF_SIZE 0x05

static uint8_t imuRXData[MPU6500_RX_BUF_SIZE];
static uint8_t imuTXData[MPU6500_TX_BUF_SIZE];

static const SPIConfig SPI5_cfg =
        {
                false,
                nullptr,
                GPIOF,
                GPIOF_SPI5_NSS,
                SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_MSTR |
                SPI_CR1_CPHA | SPI_CR1_CPOL //Set CPHA and CPOL to be 1
        };

void IMUController::start(IMUController::gyro_scale_t gyro_config, IMUController::accel_scale_t accel_config) {

    spiStart(spi_driver, &SPI5_cfg);

    uint8_t tx_frames[][2] = {
            {MPU6500_PWR_MGMT_1, MPU6500_RESET}, // reset MPU6500
            {MPU6500_PWR_MGMT_1, MPU6500_AUTO_SELECT_CLK}, // set auto clock
            {MPU6500_CONFIG, MPU6500_DLPF_41HZ}, // set bandwidth of gyro to 41Hz
            // TODO: test whether this bandwidth is suitable
            {MPU6500_GYRO_CONFIG, mpu6500_gyro_scale << 3U},
            {MPU6500_ACCEL_CONFIG, mpu6500_accel_scale << 3U},

    };




}

/**
* @brief rotate the acceleration components with the bias matrix
* @param bias matrix and acceleration components
* @return accelerate type
*/
static accelerate_t matrix3Convert(float bias[][3], float accel_x, float accel_y, float accel_z) {
    accelerate_t converted_accel;
    converted_accel.ax = bias[0][0] * accel_x + bias[0][1] * accel_y + bias[0][2] * accel_z;
    converted_accel.ay = bias[1][0] * accel_x + bias[1][1] * accel_y + bias[1][2] * accel_z;
    converted_accel.az = bias[2][0] * accel_x + bias[2][1] * accel_y + bias[2][2] * accel_z;
    return converted_accel;
}


void IMUController::IMUInit() {
    // read to set bias here
    getData();
}

void IMUController::getData() {
    uint32_t current_time = chVTGetSystemTimeX();
    dt = TIME_I2US(current_time - prev_t) / 1000000.0f;
    prev_t = current_time;
    // get dt

    // get the cofficient converting the raw data to degree
    float _gyro_psc;
    switch (_gyro_config) {
        case MPU6500_GYRO_SCALE_250:
            _gyro_psc = (1.0f / 131.0f) * PI / 180.0f;
            break;
        case MPU6500_GYRO_SCALE_500:
            _gyro_psc = (1.0f / 65.5f) * PI / 180.0f;
            break;
        case MPU6500_GYRO_SCALE_1000:
            _gyro_psc = (1.0f / 32.8f) * PI / 180.0f;
            break;
        case MPU6500_GYRO_SCALE_2000:
            _gyro_psc = (1.0f / 16.4f) * PI / 180.0f;
            break;
        default:
            return;
    }

    float _accel_psc;
    switch (_accel_config) {
        case MPU6500_ACCEL_SCALE_2G:
            _accel_psc = (GRAV / 16384.0f);
            break;
        case MPU6500_ACCEL_SCALE_4G:
            _accel_psc = (GRAV / 8192.0f);
            break;
        case MPU6500_ACCEL_SCALE_8G:
            _accel_psc = (GRAV / 4096.0f);
            break;
        case MPU6500_ACCEL_SCALE_16G:
            _accel_psc = (GRAV / 2048.0f);
            break;
        default:
            return;
    }
    /*Here to read data*/

    float accel_x = _accel_psc * (int16_t) ((imuRXData[0] << 8) | imuRXData[1]); // Accel X
    float accel_y = _accel_psc * (int16_t) ((imuRXData[2] << 8) | imuRXData[3]); // Accel Y
    float accel_z = _accel_psc * (int16_t) ((imuRXData[4] << 8) | imuRXData[5]); // Accel Z
    float gyro_x = _gyro_psc * (int16_t) ((imuRXData[8] << 8) | imuRXData[9]);  // Gyro X
    float gyro_y = _gyro_psc * (int16_t) ((imuRXData[10] << 8) | imuRXData[11]);  // Gyro Y
    float gyro_z = _gyro_psc * (int16_t) ((imuRXData[12] << 8) | imuRXData[13]);  // Gyro Z

    gyro_data.wx = (gyro_x + _gyro_bias) / dt;
    gyro_data.wy = (gyro_y + _gyro_bias) / dt;
    gyro_data.wz = (gyro_z + _gyro_bias) / dt;

    accel_data = matrix3Convert(_accel_bias, accel_x, accel_y, accel_z);
}