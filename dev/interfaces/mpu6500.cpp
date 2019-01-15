#include "mpu6500.h"
#include <cmath>

#define PI
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

void IMUController::mpu6500_write_reg(uint8_t reg_addr, uint8_t value) {
    uint8_t tx_data[2] = {reg_addr, value};
    spiAcquireBus(spi_driver);
    spiSelect(spi_driver);
    spiSend(spi_driver, 2, tx_data);
    spiUnselect(spi_driver);
    spiReleaseBus(spi_driver);
}

bool IMUController::start(IMUController::gyro_scale_t gyro_config, IMUController::accel_scale_t accel_config) {

    spiStart(spi_driver, &SPI5_cfg);
    mpu6500_write_reg(MPU6500_PWR_MGMT_1, MPU6500_RESET);
    chThdSleepMilliseconds(100);  // wait for MPU6500 to reset

    uint8_t init_reg[5][2] = {
            {MPU6500_PWR_MGMT_1, MPU6500_AUTO_SELECT_CLK}, // set auto clock
            {MPU6500_CONFIG, mpu6500_dlpf_config},
            {MPU6500_GYRO_CONFIG, mpu6500_gyro_scale << 3U},
            {MPU6500_ACCEL_CONFIG, mpu6500_accel_scale << 3U},
            {MPU6500_ACCEL_CONFIG_2, mpu6500_acc_dlpf_config}
    };

    for (int i = 0; i < 5; i++) {
        mpu6500_write_reg(init_reg[i][0], init_reg[i][1]);
    }

    return true;
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