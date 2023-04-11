//
// Created by Tianyi Han on 3/14/2023.
//

#ifndef META_EMBEDDED_IST8310_H
#define META_EMBEDDED_IST8310_H

#include "ch.hpp"
#include "hal.h"
#include "common_macro.h"
#include "ahrs_math.hpp"

#define IST8310_DATA_READY_BIT 2
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40
#define MAG_SEN 0.3f //raw int16 data change to uT unit. 原始整型数据变成 单位ut
#define IST8310_WHO_AM_I 0x00       //ist8310 "who am I "
#define IST8310_WHO_AM_I_VALUE 0x10 //device ID
#define IST8310_WRITE_REG_NUM 4

/**
 * @name IST8310Interface
 * @brief Read/write data from the IST8310 Magnetometer through the I2C bus
 * @usage 1. Call start() to enable driver and updating thread.
 * @note This interface is modified based on ChibiOS/testhal/STM32/STM32F1XX/I2C
 */
class IST8310Interface : protected chibios_rt::BaseStaticThread<512> {
public:
    /**
     * Initialize IST8310driver
     */
    chibios_rt::ThreadReference start(tprio_t prio) override;

    /**
     * Load external calibration data
     * @param mag_bias_   magnetometer bias value
     * @note To skip initial calibration at init(), call this function BEFORE init()
     */
    void load_calibration_data(Vector3D compass_bias_);

    /**
     * Return whether the device is ready
     */
    bool ready() const { return compass_startup_calibrated; };

    /**
     * Return the temperature
     */
    float get_temp() const { return temperature; }

    /**
     * Update compass
     * @note Should be called from NORMAL state (not in locks)
     */
    void update();

    Vector3D get_compass() const { return compass; }
    time_msecs_t mag_update_time = 0;   // last update time from system start [ms]

private:
    Vector3D compass_raw;   // raw (biased) data
    Vector3D compass;    // data from magnetometer
    Vector3D compass_bias;        // averaged gyro value when "static"
    Vector3D temp_compass_bias;   // temp sum of gyro for calibration
    float compass_psc;   // the coefficient converting the raw data

    const float TEMPERATURE_BIAS = 0.0f;
    float temperature;

    const float STATIC_RANGE = 0.5f; // Changes in this range is regarded as "static"
    const unsigned BIAS_SAMPLE_COUNT = 500;
    unsigned static_measurement_count; // When static_measurement_count reaches BIAS_SAMPLE_COUNT, calibration is performed.
    bool compass_startup_calibrated = false;
    time_msecs_t last_calibration_time = 0;

    static constexpr size_t RX_BUF_SIZE = 6 /* gyro */ + 2 /* temperature */ + 6 /* accel */ + 7 /* ist8310*/;
    uint8_t rx_buf[RX_BUF_SIZE];

    static constexpr int UPDATE_THREAD_INTERVAL = 1;  // [ms]

    //the first column:the registers of IST8310. 第一列:IST8310的寄存器
    //the second column: the value to be writed to the registers.第二列:需要写入的寄存器值
    //the third column: return error value.第三列:返回的错误码
    const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] ={
            {0x0B, 0x08, 0x01},     //enalbe interrupt  and low pin polarity.开启中断，并且设置低电平
            {0x41, 0x09, 0x02},     //average 2 times.平均采样两次
            {0x42, 0xC0, 0x03},     //must be 0xC0. 必须是0xC0
            {0x0A, 0x0B, 0x04}};    //200Hz output rate.200Hz输出频率

    /**
     * Initialize the IST8310
     */
    uint8_t init();

    /**
     * Main function of the thread
     */
    void main() override;

    /**
     * Write to a single register in ist8310
     * @param reg Register address
     * @param data Data to write
     */
    void ist8310_write_reg(uint8_t reg, uint8_t data);

    /**
     * Read from one or multiple registers in ist8310
     * @param reg Register address
     * @param rx_data Data to read
     * @param data_len Data length
     * @param data_offset Bytes need to be discarded before the actual data, 2 for accel, 1 for gyro
     */
    void ist8310_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t data_len, uint8_t data_offset);

    friend class AHRSCalibrationThread;
};

#endif //META_EMBEDDED_IST8310_H