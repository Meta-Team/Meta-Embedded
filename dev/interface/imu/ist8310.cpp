//
// Created by yjsxu on 2019/2/5.
//

#include "led.h"
#include "debug/shell/shell.h"
#include "ist8310.h"

/* Initialize the controller.
 * Actually, I don't know what happens here. Just copy and change properly from others.*/
bool IST8310::initial(SPIDriver *spi, IST8310::magConfigStruct conf){
    uint8_t data[5];
    data[0] = MPU6500_I2C_MST_CTRL;
    data[1] = MPU6500_I2CMST_CLK_400K; //set imu master i2c freq to 320kHz;
    data[2] = conf.addr;                    //I2C_SLV0_ADDR
    data[3] = IST8310_CTRL2;           //I2C_SLV0_REG
    data[4] = MPU6500_I2C_MSTR_READ | 6; //I2C_SLV0_CTRL
    spiAcquireBus(spi);
    spiSelect(spi);
    spiSend(spi, 5, data);
    spiUnselect(spi);
    spiReleaseBus(spi);

    data[0] = MPU6500_USER_CTRL;
    data[1] = MPU6500_USER_I2C_MST;
    spiAcquireBus(spi);
    spiSelect(spi);
    spiSend(spi, 2, data);
    spiUnselect(spi);
    spiReleaseBus(spi);

    IST8310_RESET();
    // Here is palClearPad(GPIOE,GPIOE_IST8310_RST).  Maybe reset the ist8310, but there exists error.
    chThdSleepMilliseconds(100);
    IST8310_SET();

    data[0] = MPU6500_I2C_SLV0_REG;
    data[1] = IST8310_CTRL1;
    spiAcquireBus(spi);
    spiSelect(spi);
    spiSend(spi, 2, data);
    spiUnselect(spi);
    spiReleaseBus(spi);


    data[0] = MPU6500_I2C_SLV0_DO;
    switch(conf.sample_rate)
    {
        case IST8310_SINGLE_MEASUREMENT: data[1] = 1;  break;
        case 8:                          data[1] = 2;  break;
        case 10:                         data[1] = 3;  break;
        case 20:                         data[1] = 5;  break;
        case 100:                        data[1] = 6;  break;
        case 50:                         data[1] = 7;  break;
        case IST8310_SAMPLE_RATE_1_2HZ:  data[1] = 9;  break;
        case 1:                          data[1] = 10; break;
        case 200:                        data[1] = 11; break;
        default:
            return false;
    }
    spiAcquireBus(spi);
    spiSelect(spi);
    spiSend(spi, 2, data);
    spiUnselect(spi);
    spiReleaseBus(spi);

    chThdSleepMilliseconds(2);

    data[0] = MPU6500_I2C_SLV0_ADDR;
    data[1] = conf.addr | MPU6500_I2C_MSTR_READ;
    data[2] = IST8310_XOUT_L;
    spiAcquireBus(spi);
    spiSelect(spi);
    spiSend(spi, 3, data);
    spiUnselect(spi);
    spiReleaseBus(spi);

    bias_x = 0.0f;
    bias_y = 0.0f;
    bias_z = 0.0f;

    return true;
}

void IST8310::getData(SPIDriver *spi) {
    uint16_t rawData[3];
    uint8_t data = MPU6500_EXT_SENS_DATA_00 | MPU6500_I2C_MSTR_READ;

    spiAcquireBus(spi);
    spiSelect(spi);
    spiSend(spi, 1, &data);
    spiReceive(spi, 6, (uint8_t*)rawData);
    spiUnselect(spi);
    spiReleaseBus(spi);

    magnet.x = rawData[0] * IST8310_PSC - bias_x;
    magnet.y = rawData[1] * IST8310_PSC - bias_y;
    magnet.z = rawData[2] * IST8310_PSC - bias_z;
}