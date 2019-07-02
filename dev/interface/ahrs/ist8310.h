//
// Created by yjsxu on 2019/2/5.
//

#ifndef META_INFANTRY_IST8310_H
#define META_INFANTRY_IST8310_H

#include "ch.hpp"
#include "hal.h"
#include "common_macro.h"

#include "ahrs_abstract.h"
#include "ahrs_math.hpp"
#include "math.h"

#include "mpu6500_reg.h"
#include "ist8310_reg.h"

#if defined(BOARD_RM_2018_A)
// SPI5
#define MPU6500_SPI_DRIVER SPID5
#define MPU6500_SPI_CS_PAD GPIOF
#define MPU6500_SPI_CS_PIN GPIOF_SPI5_NSS
#else
#error "MPU6500 interface has not been defined for selected board"
#endif

/**
 * @name ISTOnBoard
 * @brief Interface to get on-board IST8310 data
 * @pre MPU6500 has initialized (IST8310 use MPU6500 as I2C Master)
 * @usage 1. Call start() to enable IST8310 driver and updating thread
 *        2. Make use of data from IST8310, magnet, etc.
 */
class ISTOnBoard : virtual public AbstractIST {
public:

    /* (From AbstractIST)

    (public)
    Vector3D get_magnet();  // get magnet data [uT]
    time_msecs_t get_ist_update_time(); // get last update time from system start [ms]

    (protected)
    Vector3D magnet;  // Magnet data [uT]
    time_msecs_t ist_update_time = 0;  // Last update time from system start [ms]

    */

    /**
     * Start IST8310 driver and the thread of data fetching
     * @param prio  thread priority (recommended to be high enough)
     * @return
     */
    bool start(tprio_t prio);

    ISTOnBoard() : updateThread(*this) {};

private:

    void update();

    class UpdateThread : public chibios_rt::BaseStaticThread<512> {
    public:
        UpdateThread(ISTOnBoard& ist_) : ist(ist_) {};

    private:
        static constexpr unsigned int THREAD_UPDATE_INTERVAL = 1;  // read interval 1ms (1kHz)
        ISTOnBoard& ist;
        void main() final;
    } updateThread;


    static void writeSPIReg(const uint8_t *data, size_t n);  // helper function to write register

    enum mpu_i2cmst_clk_t{
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
    };

    static constexpr mpu_i2cmst_clk_t I2C_MASTER_CLK = MPU6500_I2CMST_CLK_400K;
};

#endif //META_INFANTRY_IST8310_H
