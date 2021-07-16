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

};

#endif //META_INFANTRY_IST8310_H
