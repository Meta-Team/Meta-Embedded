//
// Created by yjsxu on 2019/2/5.
//

#ifndef META_INFANTRY_IST8310_H
#define META_INFANTRY_IST8310_H

#include "mpu6500.h"
#include "ch.hpp"
#include "hal.h"

#define IST8310_RESET()     (palClearPad(GPIOE,GPIOE_IST8310_RST))
#define IST8310_SET()       (palSetPad(GPIOE,GPIOE_IST8310_RST))

/** IST8310 Register Maps **/
#define IST8310_STAT1      0x02
#define IST8310_XOUT_L     0x03
#define IST8310_XOUT_H     0x04
#define IST8310_YOUT_L     0x05
#define IST8310_YOUT_H     0x06
#define IST8310_ZOUT_L     0x07
#define IST8310_ZOUT_H     0x08
#define IST8310_STAT2      0x09
#define IST8310_CTRL1      0x0A
#define IST8310_CTRL2      0x0B
#define IST8310_TEMP_OUT_L 0x1C
#define IST8310_TEMP_OUT_H 0x1D

#define IST8310_STAT1_DOR  0x02
#define IST8310_STAT1_DRDY 0x01

#define IST8310_CTRL2_DREN 0x08
#define IST8310_CTRL2_DRP  0x04
#define IST8310_CTRL2_SRST 0x01

#define IST8310_PSC 0.003f
#define IST8310_SINGLE_MEASUREMENT    0
#define IST8310_SAMPLE_RATE_1_2HZ   255

typedef enum {
    MPU6500_I2CMST_CLK_348K = 0,
    MPU6500_I2CMST_CLK_333K = 1,
    MPU6500_I2CMST_CLK_320K = 2,
    MPU6500_I2CMST_CLK_308K = 3,
    MPU6500_I2CMST_CLK_296K = 4,
    MPU6500_I2CMST_CLK_286K = 5,
    MPU6500_I2CMST_CLK_276K = 6,
    MPU6500_I2CMST_CLK_267K = 7,
    MPU6500_I2CMST_CLK_258K = 8,
    MPU6500_I2CMST_CLK_500K = 9,
    MPU6500_I2CMST_CLK_471K = 10,
    MPU6500_I2CMST_CLK_444K = 11,
    MPU6500_I2CMST_CLK_421K = 12,
    MPU6500_I2CMST_CLK_400K = 13,
    MPU6500_I2CMST_CLK_381K = 14,
    MPU6500_I2CMST_CLK_364K = 15
} mpu_i2cmst_clk_t;

class IST8310Controller{
public:

    typedef enum{
        IST8310_ADDR_FLOATING = 0x0E,
        IST8310_ADDR_0_0      = 0x0C,
        IST8310_ADDR_0_1      = 0x0D,
        IST8310_ADDR_1_0      = 0x0E,
        IST8310_ADDR_1_1      = 0x0F

    } ist8310_i2c_addr_t;

    typedef struct{
        ist8310_i2c_addr_t addr;
        uint8_t sample_rate;
    }magConfigStruct;

    static Vector3D magnet;
    static bool initial(SPIDriver *spi, magConfigStruct conf);
    static void getData(SPIDriver *spi, )

private:
    static SPIDriver *spi_driver;
    static float bias_x;
    static float bias_y;
    static float bias_z;
};

#endif //META_INFANTRY_IST8310_H
