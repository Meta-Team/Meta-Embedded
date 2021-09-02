#include "imu_on_board.h"

#include "led.h"
#include "mpu6500_reg.h"
#include "ist8310_reg.h"

static const SPIConfig SPI5_cfg =
        {
                false,
                nullptr,
                MPU6500_SPI_CS_PAD,
                MPU6500_SPI_CS_PIN,
                SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_MSTR |
                SPI_CR1_CPHA | SPI_CR1_CPOL, //Set CPHA and CPOL to be 1
                0
        };

/**
 * @note SPI Read/Write Protocol
 * 1. (ChibiOS) Lock the SPI driver (spiAcquireBus)
 * 2. CS (Chip Select) down (spiSelect)
 * 3. Send register address N (bit 7 is 0 for write, 1 for read)
 * 4. Read/Write register N
 * (Optional, repeated) Read/Write register N+1, N+2, N+3, ...
 * 5. CS up (spiUnselect)
 * 6. (ChibiOS) unlock the SPI driver (spiReleaseBus)
 */

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

/**
 * Write a single register through SPI.
 * @param reg_addr The register
 * @param value    The value to write
 */
void mpu6500_write_reg(uint8_t reg_addr, uint8_t value) {
    uint8_t tx_data[2] = {reg_addr /* bit 7 is keep as 0 for write */, value};
    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, 2, tx_data);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);
}


chibios_rt::ThreadReference IMUOnBoard::start(tprio_t prio) {
    init_mpu6500();
    init_ist8310();  // should be after initialization of MPU6500
    return chibios_rt::BaseStaticThread<512>::start(prio);
}

void IMUOnBoard::init_mpu6500() {
    // Start SPI driver
    spiStart(&MPU6500_SPI_DRIVER, &SPI5_cfg);

    // Reset MPU6500
    mpu6500_write_reg(MPU6500_PWR_MGMT_1, MPU6500_RESET);
    chThdSleepMilliseconds(100);  // wait for MPU6500 to reset, see data sheet

    // Enable MPU6500 as I2C Master (for IST8310) and disable as I2C Slave
    // MPU6500_I2C_IF_DIS disable MPU6500 as I2C Slave and enable SPI only, by datasheet p.30 6.1 Note
    mpu6500_write_reg(MPU6500_USER_CTRL, MPU6500_I2C_MST_EN | MPU6500_I2C_IF_DIS);

    // Configure MPU6500 for gyro and accel
    uint8_t init_reg[5][2] = {
            {MPU6500_PWR_MGMT_1, MPU6500_AUTO_SELECT_CLK},  // auto clock
            {MPU6500_CONFIG,         CONFIG._dlpf_config},
            {MPU6500_GYRO_CONFIG,    CONFIG._gyro_scale << 3U /* [1:0] for FCHOICE_B = b00, low-pass-filter enabled */},
            {MPU6500_ACCEL_CONFIG,   CONFIG._accel_scale << 3U},
            {MPU6500_ACCEL_CONFIG_2, CONFIG._acc_dlpf_config}
    };
    for (auto & i : init_reg) {
        mpu6500_write_reg(i[0], i[1]);
        chThdSleepMilliseconds(10);
    }

    temperature = 0;

    // Get the coefficient converting the raw data to degree or gravity
    switch (CONFIG._gyro_scale) {
        case MPU6500_GYRO_SCALE_250:
            gyro_psc = (1.0f / 131.0f);
            break;
        case MPU6500_GYRO_SCALE_500:
            gyro_psc = (1.0f / 65.5f);
            break;
        case MPU6500_GYRO_SCALE_1000:
            gyro_psc = (1.0f / 32.8f);
            break;
        case MPU6500_GYRO_SCALE_2000:
            gyro_psc = (1.0f / 16.4f);
            break;
        default:
            gyro_psc = 0.0f;
            break;
    }

    switch (CONFIG._accel_scale) {
        case MPU6500_ACCEL_SCALE_2G:
            accel_psc = (1 / 16384.0f) * GRAV_CONSTANT;
            break;
        case MPU6500_ACCEL_SCALE_4G:
            accel_psc = (1 / 8192.0f) * GRAV_CONSTANT;
            break;
        case MPU6500_ACCEL_SCALE_8G:
            accel_psc = (1 / 4096.0f) * GRAV_CONSTANT;
            break;
        case MPU6500_ACCEL_SCALE_16G:
            accel_psc = (1 / 2048.0f) * GRAV_CONSTANT;
            break;
        default:
            accel_psc = 0.0f;
            break;
    }
}

void IMUOnBoard::init_ist8310() {

    // Reset IST8310
    palClearPad(GPIOE, GPIOE_IST8310_RST);
    chThdSleepMilliseconds(100);
    palSetPad(GPIOE, GPIOE_IST8310_RST);

    // Setup IST8310 through MPU6500 as I2C Master

    /*
     * Here we set up MPU6500 as the I2C Master to IST8310. We control MPU6500 through SPI (see mpu6500.c for SPI
     * protocol), which indirectly controls IST8310 through its I2C. The process of indirect read/write:
     *  1. Write the Slave register address to I2C_SLVX_ADDR (SLVX can be SLV0, SLV1, up to SLV4). If read, bit 7
     *     should be set (MPU6500_I2C_MSTR_READ).
     *  2. (For write) write the data to I2C_SLVX_DO.
     *  3. Enable read/write by I2C_SLV4_CTRL
     *  4. (For read) read data from MPU6500_EXT_SENS_DATA_XX.
     *
     *
     * To use IST8310, we need to configure IST8310 by writing some data to its register. We use indirectly writes.
     *
     * Then, we ask MPU6500 to fetch data automatically through I2C. And we then fetch the data from it through SPI.
     * Recommended IST8310 Read Process from datasheet p.9 3.4:
     *  Read STAT1 register.
     *  Read Measurement Data in 6 registers.
     * STAT1 is right before the measurement data registers, so we ask MPU6500 to fetch 7 registers
     * (MPU6500_I2C_SLV_READ_7) starting from STAT1.
     * Data of these 7 registers are stored starting from MPU6500_EXT_SENS_DATA_00. In update thread, we start reading
     * from MPU6500_EXT_SENS_DATA_01 to skip the STAT1 register.
     *
     * By default MPU6500 considers Slave register 0 and 1, 2 and 3, etc. as a word. If byte-swapping in
     * I2C_SLVX_CTRL is enabled, the two bytes in each word is swap. As we start reading from STAT1, which is a
     * standalone register, we want to ask MPU6500 to group 1 and 2 (X), 3 and 4 (Y), 5 and 6 (Z). Therefore, we
     * set MPU6500_I2C_SLV_GRP. However, this bit is (very likely) useless since byte-swapping is not enabled.
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
            {IST8310_PDCTNL,  IST8310_PULSE_DURATION_NORMAL},  // recommended by datasheet p.8 3.1.1
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
    data[0] = MPU6500_I2C_MST_CTRL | MPU6500_SPI_WRITE;     // start writing from MPU6500_I2C_MST_CTRL (36)
    data[1] = MPU6500_I2CMST_CLK_400K;                      // reg 36: I2C Master Control, use 400KHz I2C to Slave
    data[2] = IST8310_IIC_ADDRESS | MPU6500_I2C_MSTR_READ;  // reg 37: I2C_SLV0_ADDR, to read from I2C addr 0x0E
    data[3] = IST8310_STAT1;                                // reg 38: I2C_SLV0_REG, to start read from reg STAT1
    data[4] = MPU6500_I2C_SLV_EN | MPU6500_I2C_SLV_GRP | MPU6500_I2C_SLV_READ_7;  // reg 39: I2C_SLV0_CTRL
    // Enable reading 7 registers. See the paragraph above for MPU6500_I2C_SLV_GRP
    write_spi_reg(data, 5);
}

void IMUOnBoard::load_calibration_data(Vector3D gyro_bias_) {
    gyro_bias = gyro_bias_;
    last_calibration_time = SYSTIME;
}

void IMUOnBoard::update() {

    // Fetch data from SPI
    uint8_t tx_data = MPU6500_ACCEL_XOUT_H | MPU6500_SPI_READ;
    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, 1, &tx_data);
    spiReceive(&MPU6500_SPI_DRIVER, RX_BUF_SIZE, rx_buf);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);

    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    {

        /// Decode data

        // MPU6500 stores high byte first
        Vector3D accel_raw = Vector3D((int16_t) (rx_buf[0] << 8 | rx_buf[1]),
                             (int16_t) (rx_buf[2] << 8 | rx_buf[3]),
                             (int16_t) (rx_buf[4] << 8 | rx_buf[5])) * accel_psc;
        temperature = ((((int16_t) (rx_buf[6] << 8 | rx_buf[7])) - TEMPERATURE_BIAS) / 333.87f) + 21.0f;
        Vector3D new_gyro_raw = Vector3D((int16_t) (rx_buf[8] << 8 | rx_buf[9]),
                                         (int16_t) (rx_buf[10] << 8 | rx_buf[11]),
                                         (int16_t) (rx_buf[12] << 8 | rx_buf[13])) * gyro_psc;

        // IST8310 stores low byte first
        // rx_buf[14] is STAT1, see init_ist8310() for details
        magnet.x = (float) (int16_t) (rx_buf[16] << 8 | rx_buf[15]) * IST8310_PSC;
        magnet.y = (float) (int16_t) (rx_buf[18] << 8 | rx_buf[17]) * IST8310_PSC;
        magnet.z = (float) (int16_t) (rx_buf[20] << 8 | rx_buf[19]) * IST8310_PSC;

        /// Gyro Calibration sampling

        if ((ABS_IN_RANGE(new_gyro_raw.x - gyro_raw.x, STATIC_RANGE) &&
             ABS_IN_RANGE(new_gyro_raw.y - gyro_raw.y, STATIC_RANGE) &&
             ABS_IN_RANGE(new_gyro_raw.z - gyro_raw.z, STATIC_RANGE)) &&
            !imu_startup_calibrated) {  // MPU6500 static

            static_measurement_count++;
            temp_gyro_bias = temp_gyro_bias - new_gyro_raw;  // bias is to be minus, so sum as negative here
        }

        /// Bias data

        gyro_raw = new_gyro_raw;
        gyro = gyro_raw + gyro_bias;
        accel = accel_raw;

        /// Update info

        imu_update_time = SYSTIME;

        /// Perform gyro re-bias

        if (!imu_startup_calibrated && (static_measurement_count >= BIAS_SAMPLE_COUNT) && (SYSTIME > MPU6500_STARTUP_TIME)) {

            imu_startup_calibrated = true;
            gyro_bias = temp_gyro_bias / static_measurement_count;
            static_measurement_count = 0;
            temp_gyro_bias = Vector3D(0, 0, 0);
            last_calibration_time = SYSTIME;
        }

    }
    chSysUnlock();  /// --- EXIT S-Locked state ---
}

void IMUOnBoard::main() {
    setName("IMU");
    while (!shouldTerminate()) {
        update();
        sleep(TIME_MS2I(UPDATE_THREAD_INTERVAL));
    }
}
