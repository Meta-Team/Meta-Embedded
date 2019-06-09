//
// Created by yjsxu on 2019/2/5.
//

#include <interface/led.h>
#include "ist8310.h"
#include "math.h"

Vector3D IST8310::magnet;
matrix3 IST8310::magnet_bias;
IST8310::IST8310UpdateThread IST8310::updateThread;

/* Initialize the controller.
 * Actually, I don't know what happens here. Just copy and change properly from others.*/

bool IST8310::start(tprio_t prio){

    palClearPad(GPIOE, GPIOE_IST8310_RST);
    chThdSleepMilliseconds(100);
    palSetPad(GPIOE, GPIOE_IST8310_RST);

    // TODO: add WHO I AM

    uint8_t data[5];
    data[0] = MPU6500_I2C_MST_CTRL;
    data[1] = MPU6500_I2CMST_CLK_400K;                     // I2C Master Control
    data[2] = IST8310_IIC_ADDRESS | MPU6500_I2C_MSTR_READ; // I2C_SLV0_ADDR
    data[3] = 0x02;                                        // I2C_SLV0_REG
    data[4] = 0x88;                                        // I2C_SLV0_CTRL
    writeSPIReg(data, 5);

    data[0] = MPU6500_USER_CTRL;
    data[1] = MPU6500_USER_I2C_MST | 0x10;
    writeSPIReg(data, 2);

//    data[0] = MPU6500_I2C_SLV0_REG;
//    data[1] = IST8310_CTRL1;
//    writeSPIReg(data, 2);

    data[0] = MPU6500_I2C_SLV4_ADDR;
    data[1] = IST8310_IIC_ADDRESS;
    data[4] = MPU6500_SPI_READ;

    uint8_t init_reg[4][2] = {
            {0x0B, 0x08},
            {0x41, 0x09},
            {0x42, 0xC0},
            {0x0A, 0x0B}
    };


    for (int i = 0; i < 4; i++) {
        data[2] = init_reg[i][0];
        data[3] = init_reg[i][1];
        writeSPIReg(data, 5);
        chThdSleepMilliseconds(10);
    }

//    float temp_m_bias_x = 0, temp_m_bias_y = 0, temp_m_bias_z = 0;
//
//    for (int i = 0; i < IST8310_BIAS_SAMPLE_COUNT; i++) {
//        getData();
//        temp_m_bias_x += magnet.x;
//        temp_m_bias_y += magnet.y;
//        temp_m_bias_z += magnet.z;
//    }
//
//    magnet_bias[2][0] = temp_m_bias_x / IST8310_BIAS_SAMPLE_COUNT;
//    magnet_bias[2][1] = temp_m_bias_x / IST8310_BIAS_SAMPLE_COUNT;
//    magnet_bias[2][2] = temp_m_bias_x / IST8310_BIAS_SAMPLE_COUNT;
//    magnet_bias[0][0] = magnet_bias[2][1] - magnet_bias[2][2];
//    magnet_bias[0][1] = magnet_bias[2][2] - magnet_bias[2][0];
//    magnet_bias[0][0] = magnet_bias[2][0] - magnet_bias[2][1];
//    float length = sqrt(magnet_bias[0][0] * magnet_bias[0][0] + magnet_bias[0][1] * magnet_bias[0][1]
//                        + magnet_bias[0][2] * magnet_bias[0][2]);
//    magnet_bias[0][0] /= length;
//    magnet_bias[0][1] /= length;
//    magnet_bias[0][2] /= length;
//    Vector3D temp_vect = Vector3D(magnet_bias[0]).crossMultiply(Vector3D(magnet_bias[2]));
//    magnet_bias[1][0] = temp_vect.x;
//    magnet_bias[1][1] = temp_vect.y;
//    magnet_bias[1][2] = temp_vect.z;

//    bias_x = 0.0f;
//    bias_y = 0.0f;
//    bias_z = 0.0f;

    // Start the update thread
    updateThread.start(prio);

    return true;
}

void IST8310::writeSPIReg(const uint8_t *data, size_t n) {
    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, n, data);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);
}

void IST8310::getData() {

    int16_t rawData[3];
    uint8_t data = MPU6500_EXT_SENS_DATA_00 | MPU6500_I2C_MSTR_READ;

    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, 1, &data);
    spiReceive(&MPU6500_SPI_DRIVER, 6, (uint8_t*)rawData);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);

    magnet.x = (float) rawData[0] * IST8310_PSC;
    magnet.y = (float) rawData[1] * IST8310_PSC;
    magnet.z = (float) rawData[2] * IST8310_PSC;
//    magnet = Vector3D(rawData[0] * IST8310_PSC, rawData[1] * IST8310_PSC, rawData[2] * IST8310_PSC) * magnet_bias;
}