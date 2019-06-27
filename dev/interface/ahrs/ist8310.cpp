//
// Created by yjsxu on 2019/2/5.
//

#include "ist8310.h"

bool ISTOnBoard::start(tprio_t prio) {

    palClearPad(GPIOE, GPIOE_IST8310_RST);
    chThdSleepMilliseconds(100);
    palSetPad(GPIOE, GPIOE_IST8310_RST);

    uint8_t data[5];
    data[0] = MPU6500_I2C_MST_CTRL;
    data[1] = I2C_MASTER_CLK;                              // I2C Master Control
    data[2] = IST8310_IIC_ADDRESS | MPU6500_I2C_MSTR_READ; // I2C_SLV0_ADDR
    data[3] = 0x02;                                        // I2C_SLV0_REG
    data[4] = 0x88;                                        // I2C_SLV0_CTRL
    writeSPIReg(data, 5);

    data[0] = MPU6500_USER_CTRL;
    data[1] = MPU6500_USER_I2C_MST | 0x10;
    writeSPIReg(data, 2);


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

    // Start the update thread
    updateThread.start(prio);

    return true;
}

void ISTOnBoard::writeSPIReg(const uint8_t *data, size_t n) {
    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, n, data);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);
}


void ISTOnBoard::update() {

    int16_t rawData[3];
    uint8_t data = MPU6500_EXT_SENS_DATA_00 | MPU6500_I2C_MSTR_READ;

    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, 1, &data);
    spiReceive(&MPU6500_SPI_DRIVER, 6, (uint8_t *) rawData);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);

    magnet.x = (float) rawData[0] * IST8310_PSC;
    magnet.y = (float) rawData[1] * IST8310_PSC;
    magnet.z = (float) rawData[2] * IST8310_PSC;

    ist_update_time = SYSTIME;
    
}


void ISTOnBoard::UpdateThread::main() {
    setName("IST8310");
    while (!shouldTerminate()) {

        ist.update();

        sleep(TIME_MS2I(THREAD_UPDATE_INTERVAL));
    }
}