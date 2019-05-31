#include "mpu6500.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "math.h"

#define GRAV 9.80665f
#define MPU6500_RX_BUF_SIZE 0x0E
#define TEMP_OFFSET 0.0f

#define MPU6500_BIAS_SAMPLE_COUNT 50
#define MPU6500_BIAS_SAMPLE_INTERVAL 10 // [ms]

MPU6500::MPU6500UpdateThread MPU6500::updateThread;

Vector3D MPU6500::angle_speed;  // final data of gyro
Vector3D MPU6500::acceleration;  // final data of acceleration
float MPU6500::temperature;
time_msecs_t MPU6500::last_update_time;

float MPU6500::dt;
float MPU6500::prev_t;

float MPU6500::_gyro_psc;  // get the coefficient converting the raw data to degree
float MPU6500::_accel_psc;  // get the coefficient converting the raw data to g

Vector3D MPU6500::_gyro_bias;  // for gyro bias
matrix3 MPU6500::_accel_bias;  // a matrix for accelerate bias

uint8_t mpu6500_RXData[MPU6500_RX_BUF_SIZE];

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

void MPU6500::mpu6500_write_reg(uint8_t reg_addr, uint8_t value) {
    uint8_t tx_data[2] = {reg_addr, value};
    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, 2, tx_data);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);
}

bool MPU6500::start(tprio_t prio) {

    spiStart(&MPU6500_SPI_DRIVER, &SPI5_cfg);
    mpu6500_write_reg(MPU6500_PWR_MGMT_1, MPU6500_RESET);
    chThdSleepMilliseconds(100);  // wait for MPU6500 to reset, see data sheet


    uint8_t init_reg[5][2] = {
            {MPU6500_PWR_MGMT_1, MPU6500_AUTO_SELECT_CLK}, // set auto clock
            {MPU6500_CONFIG, config._dlpf_config},
            {MPU6500_GYRO_CONFIG, config._gyro_scale << 3U},
            {MPU6500_ACCEL_CONFIG, config._accel_scale << 3U},
            {MPU6500_ACCEL_CONFIG_2, config._acc_dlpf_config}
    };

    for (int i = 0; i < 5; i++) {
        mpu6500_write_reg(init_reg[i][0], init_reg[i][1]);
        chThdSleepMilliseconds(10);
    }

    dt = 0;
    temperature = 0;
    _accel_bias[0][0] = 1.0f; _accel_bias[0][1] = 0.0f; _accel_bias[0][2] = 0.0f;
    _accel_bias[1][0] = 0.0f; _accel_bias[1][1] = 1.0f; _accel_bias[1][2] = 0.0f;
    _accel_bias[2][0] = 0.0f; _accel_bias[2][1] = 0.0f; _accel_bias[2][2] = 1.0f;

    // get the coefficient converting the raw data to degree or gravity
    switch(config._gyro_scale)
    {
        case MPU6500_GYRO_SCALE_250:
            _gyro_psc = (1.0f / 131.0f);
            break;
        case MPU6500_GYRO_SCALE_500:
            _gyro_psc = (1.0f /  65.5f);
            break;
        case MPU6500_GYRO_SCALE_1000:
            _gyro_psc = (1.0f /  32.8f);
            break;
        case MPU6500_GYRO_SCALE_2000:
            _gyro_psc = (1.0f /  16.4f);
            break;
        default:
            _gyro_psc = 0.0f;
            break;
    }

    switch(config._accel_scale)
    {
        case MPU6500_ACCEL_SCALE_2G:
            _accel_psc = (1 / 16384.0f);
            break;
        case MPU6500_ACCEL_SCALE_4G:
            _accel_psc = (1 /  8192.0f);
            break;
        case MPU6500_ACCEL_SCALE_8G:
            _accel_psc = (1 /  4096.0f);
            break;
        case MPU6500_ACCEL_SCALE_16G:
            _accel_psc = (1 /  2048.0f);
            break;
        default:
            _accel_psc = 0.0f;
            break;
    }

    float temp_g_bias_x = 0, temp_g_bias_y = 0, temp_g_bias_z = 0;
    float temp_a_bias_x = 0, temp_a_bias_y = 0, temp_a_bias_z = 0;

    for (int i = 0; i < MPU6500_BIAS_SAMPLE_COUNT; i++) {
        getData();
        temp_g_bias_x -= angle_speed.x;
        temp_g_bias_y -= angle_speed.y;
        temp_g_bias_z -= angle_speed.z;
        temp_a_bias_x += acceleration.x;
        temp_a_bias_y += acceleration.y;
        temp_a_bias_z += acceleration.z;
        chThdSleepMilliseconds(MPU6500_BIAS_SAMPLE_INTERVAL);
    }

    _gyro_bias.x = temp_g_bias_x / MPU6500_BIAS_SAMPLE_COUNT;
    _gyro_bias.y = temp_g_bias_y / MPU6500_BIAS_SAMPLE_COUNT;
    _gyro_bias.z = temp_g_bias_z / MPU6500_BIAS_SAMPLE_COUNT;

    _accel_bias[2][0] = temp_a_bias_x / MPU6500_BIAS_SAMPLE_COUNT;
    _accel_bias[2][1] = temp_a_bias_y / MPU6500_BIAS_SAMPLE_COUNT;
    _accel_bias[2][2] = temp_a_bias_z / MPU6500_BIAS_SAMPLE_COUNT;
    _accel_bias[0][0] = _accel_bias[2][1] - _accel_bias[2][2];
    _accel_bias[0][1] = _accel_bias[2][2] - _accel_bias[2][0];
    _accel_bias[0][0] = _accel_bias[2][0] - _accel_bias[2][1];
    float length = sqrt(_accel_bias[0][0] * _accel_bias[0][0] + _accel_bias[0][1] * _accel_bias[0][1]
                        + _accel_bias[0][2] * _accel_bias[0][2]);
    _accel_bias[0][0] /= length;
    _accel_bias[0][1] /= length;
    _accel_bias[0][2] /= length;
    Vector3D temp_vect = Vector3D(_accel_bias[0]).crossMultiply(Vector3D(_accel_bias[2]));
    _accel_bias[1][0] = temp_vect.x;
    _accel_bias[1][1] = temp_vect.y;
    _accel_bias[1][2] = temp_vect.z;

    // Start the update thread
    updateThread.start(prio);

    return true;
}

void MPU6500::getData() {
    uint32_t current_time =  chibios_rt::System::getTime();
    dt = TIME_I2MS(current_time - prev_t) / 1000.0f;
    prev_t = current_time;

    // Acquire data
    uint8_t tx_data = MPU6500_ACCEL_XOUT_H | MPU6500_SPI_READ;
    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, 1, &tx_data);
    chThdSleepMilliseconds(1);
    spiReceive(&MPU6500_SPI_DRIVER, MPU6500_RX_BUF_SIZE, mpu6500_RXData);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);


//    chSysLock();  // --- Enter Critical Zone ---

    float accel_x = _accel_psc * (int16_t)((mpu6500_RXData[ 0]<<8) | mpu6500_RXData[ 1]); // Accel X
    float accel_y = _accel_psc * (int16_t)((mpu6500_RXData[ 2]<<8) | mpu6500_RXData[ 3]); // Accel Y
    float accel_z = _accel_psc * (int16_t)((mpu6500_RXData[ 4]<<8) | mpu6500_RXData[ 5]); // Accel Z
    float gyro_x = _gyro_psc * (int16_t)((mpu6500_RXData[ 8]<<8) | mpu6500_RXData[ 9]);  // Gyro X
    float gyro_y = _gyro_psc * (int16_t)((mpu6500_RXData[10]<<8) | mpu6500_RXData[11]);  // Gyro Y
    float gyro_z = _gyro_psc * (int16_t)((mpu6500_RXData[12]<<8) | mpu6500_RXData[13]);  // Gyro Z
    auto temper = (int16_t)((mpu6500_RXData[6]<<8) | mpu6500_RXData[7]); // Temperature

    temperature = (((float)temper - TEMP_OFFSET) / 333.87f) + 21.0f;

    angle_speed.x = (gyro_x + _gyro_bias.x);
    angle_speed.y = (gyro_y + _gyro_bias.y);
    angle_speed.z = (gyro_z + _gyro_bias.z);

    acceleration = Vector3D(accel_x, accel_y, accel_z) * _accel_bias;

    last_update_time = SYSTIME;

//    chSysUnlock();  // --- Exit Critical Zone ---
}