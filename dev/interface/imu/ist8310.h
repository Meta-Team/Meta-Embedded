//
// Created by yjsxu on 2019/2/5.
//

#ifndef META_INFANTRY_IST8310_H
#define META_INFANTRY_IST8310_H

#include "ch.hpp"
#include "hal.h"

#include "ahrs_math.hpp"
#include "mpu6500_reg.h"
#include "ist8310_reg.h"

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

#define IST8310_BIAS_SAMPLE_COUNT 500

class IST8310 {
public:

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

    typedef enum{
        IST8310_ADDR_FLOATING = 0x0E,
        IST8310_ADDR_0_0      = 0x0C,
        IST8310_ADDR_0_1      = 0x0D,
        IST8310_ADDR_1_0      = 0x0E,
        IST8310_ADDR_1_1      = 0x0F

    } ist8310_i2c_addr_t;

    static Vector3D magnet;

    static bool start(tprio_t prio);
    static void getData();

private:


    static constexpr unsigned int IST8310_THREAD_UPDATE_INTERVAL = 1;  // read interval 1ms (1kHz)

    class IST8310UpdateThread : public chibios_rt::BaseStaticThread<512> {
        void main() final {
            setName("ist8310");
            while (!shouldTerminate()) {
                IST8310::getData();
                sleep(TIME_MS2I(IST8310_THREAD_UPDATE_INTERVAL));
            }
        }
    };

    static IST8310UpdateThread updateThread;

    static void writeSPIReg(const uint8_t *data, size_t n);
};

#endif //META_INFANTRY_IST8310_H
