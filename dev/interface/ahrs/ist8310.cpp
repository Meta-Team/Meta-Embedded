//
// Created by yjsxu on 2019/2/5.
//

#include "ist8310.h"

/**
 * Write one or more register(s) through SPI
 * @param data [(register address N | write bit), data for register N, data for register N+1, ...]
 * @param n    Data array length
 */
void write_spi_reg(const uint8_t *data, size_t n) {
    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, n, data);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);
}

bool ISTOnBoard::start(tprio_t prio) {

    // Reset IST8310
    palClearPad(GPIOE, GPIOE_IST8310_RST);
    chThdSleepMilliseconds(100);
    palSetPad(GPIOE, GPIOE_IST8310_RST);

    // Setup IST8310 through MPU6500 as I2C Master

    /*
     * Here we set up MPU6500 as the I2C Master to IST8310. We control MPU6500 through SPI (see mpu6500.c for SPI
     * protocol), which indirectly control IST8310 through its I2C.
     *
     * For the first step, we need to configure IST8310 by writing some data to its register. We use indirectly writes.
     * That is, we set up MPU6500 as the I2C Master, write the IST8310's register and data to MPU6500's registers, and
     * ask MPU6500 to send the data.
     *
     * Then, we ask MPU6500 to fetch data automatically through I2C. And we then fetch the data from it through SPI.
     * Recommended IST8310 Read Process from datasheet p.9 3.4:
     *  Read STAT1 register.
     *  Read Measurement Data in 6 registers.
     * STAT1 is right before the measurement data registers, so we ask MPU6500 to fetch 7 registers
     * (MPU6500_I2C_SLV_READ_7) starting from STAT1.
     */

    uint8_t data[5];

    // Configure MPU6500 as I2C Master of IST8310 as Slave 4
    // We use Slave 4 for setting as its Data Out reg is right before CTRL register, allowing burst write
    data[0] = MPU6500_I2C_SLV4_ADDR | MPU6500_SPI_WRITE;     // start writing from I2C_SLV4_ADDR
    data[1] = IST8310_IIC_ADDRESS | MPU6500_I2C_MSTR_WRITE;  // reg 49: I2C_SLV4_ADDR, to write to IST8310 I2C addr
    data[2] = 0; /* to be filled below*/                     // reg 50: I2C_SLV4_REG, IST8310 reg to write to
    data[3] = 0; /* to be filled below*/                     // reg 51: I2C_SLV4_DO, data to write to IST8310 reg
    data[4] = MPU6500_I2C_SLV_EN;                            // reg 52: I2C_SLV4_CTRL, enable data transfer

    uint8_t init_reg[4][2] = {
            {IST8310_CTRL2,   IST8310_CTRL2_DREN},             // FIXME: unknown
            {IST8310_AVGCNTL, IST8310_X_AND_Z_AVG_2_TIMES | IST8310_Y_AVG_2_TIMES},
            {IST8310_PDCTNL,  IST8310_PULSE_DURATION_NORMAL},  // recommended by datasheet on p.8 3.1.1
            {IST8310_CTRL1,   IST8310_CONTINUOUS_200HZ}        // see IST8310_CONTINUOUS_200HZ definition
    };
    for (auto &i : init_reg) {
        data[2] = i[0];
        data[3] = i[1];
        write_spi_reg(data, 5);
        chThdSleepMilliseconds(10);
    }

    // Set MPU6500 as I2C master of IST8310 as Slave 0
    // We use Slave 0 for reading as its Data In regs are right after gyro data regs, allowing burst read
    data[0] = MPU6500_I2C_MST_CTRL | MPU6500_SPI_WRITE;    // start writing from MPU6500_I2C_MST_CTRL (36)
    data[1] = MPU6500_I2CMST_CLK_400K;                     // reg 36: I2C Master Control, use 400KHz I2C to Slave
    data[2] = IST8310_IIC_ADDRESS | MPU6500_I2C_MSTR_READ; // reg 37: I2C_SLV0_ADDR, to read from I2C addr 0x0E
    data[3] = IST8310_STAT1;                               // reg 38: I2C_SLV0_REG, to start read from reg STAT1
    data[4] = MPU6500_I2C_SLV_EN | MPU6500_I2C_SLV_GRP | MPU6500_I2C_SLV_READ_7;  // reg 39: I2C_SLV0_CTRL
    write_spi_reg(data, 5);

    // Enable MPU6500
    data[0] = MPU6500_USER_CTRL;
    data[1] = MPU6500_USER_I2C_MST | 0x10;
    write_spi_reg(data, 2);


    // Start the update thread
    updateThread.start(prio);

    return true;
}


void ISTOnBoard::update() {
    int16_t rawData[3];
    uint8_t data = MPU6500_EXT_SENS_DATA_01 | MPU6500_I2C_MSTR_READ;  // FIXME: data start

    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, 1, &data);
    spiReceive(&MPU6500_SPI_DRIVER, 6, (uint8_t *) rawData);  // little endian
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);

    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    {
        magnet.x = (float) rawData[0] * IST8310_PSC;
        magnet.y = (float) rawData[1] * IST8310_PSC;
        magnet.z = (float) rawData[2] * IST8310_PSC;
    }
    chSysUnlock();  /// --- EXIT S-Locked state ---
    ist_update_time = SYSTIME;

}


void ISTOnBoard::UpdateThread::main() {
    setName("IST8310");
    while (!shouldTerminate()) {
        ist.update();
        sleep(TIME_MS2I(THREAD_UPDATE_INTERVAL));
    }
}