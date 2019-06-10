#include "mpu6500.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "math.h"

#define GRAV 9.80665f
#define MPU6500_RX_BUF_SIZE 0x0E
#define TEMP_OFFSET 0.0f

#define IST8310_BIAS_SAMPLE_COUNT 1000
#define MPU6500_BIAS_SAMPLE_INTERVAL 10 // [ms]

MPU6500::UpdateThread MPU6500::updateThread;

Vector3D MPU6500::gyro_orig;
Vector3D MPU6500::gyro;  // final data of gyro
Vector3D MPU6500::accel_orig;
#if MPU6500_ENABLE_ACCEL_BIAS
Vector3D MPU6500::accel;  // final data of acceleration
#endif
float MPU6500::temperature;
time_msecs_t MPU6500::last_update_time = 0;

float MPU6500::gyro_psc;   // get the coefficient converting the raw data to degree
float MPU6500::accel_psc;  // get the coefficient converting the raw data to g

unsigned MPU6500::static_measurement_count = 0;
time_msecs_t MPU6500::last_calibration_time = 0;

Vector3D MPU6500::gyro_bias;  // for gyro bias
#if MPU6500_ENABLE_ACCEL_BIAS
Matrix33 MPU6500::accel_bias;  // a matrix for accelerate bias
#endif
Vector3D MPU6500::temp_gyro_bias;  // for gyro bias


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
            {MPU6500_CONFIG,         CONFIG._dlpf_config},
            {MPU6500_GYRO_CONFIG,    CONFIG._gyro_scale << 3U},
            {MPU6500_ACCEL_CONFIG,   CONFIG._accel_scale << 3U},
            {MPU6500_ACCEL_CONFIG_2, CONFIG._acc_dlpf_config}
    };

    for (int i = 0; i < 5; i++) {
        mpu6500_write_reg(init_reg[i][0], init_reg[i][1]);
        chThdSleepMilliseconds(10);
    }

    temperature = 0;

    // Get the coefficient converting the raw data to degree or gravity
    switch (CONFIG._gyro_scale) {
        case MPU6500_GYRO_SCALE_250:  gyro_psc = (1.0f / 131.0f);  break;
        case MPU6500_GYRO_SCALE_500:  gyro_psc = (1.0f / 65.5f);   break;
        case MPU6500_GYRO_SCALE_1000: gyro_psc = (1.0f / 32.8f);   break;
        case MPU6500_GYRO_SCALE_2000: gyro_psc = (1.0f / 16.4f);   break;
        default:                      gyro_psc = 0.0f;             break;
    }

    switch (CONFIG._accel_scale) {
        case MPU6500_ACCEL_SCALE_2G:  accel_psc = (1 / 16384.0f); break;
        case MPU6500_ACCEL_SCALE_4G:  accel_psc = (1 / 8192.0f);  break;
        case MPU6500_ACCEL_SCALE_8G:  accel_psc = (1 / 4096.0f);  break;
        case MPU6500_ACCEL_SCALE_16G: accel_psc = (1 / 2048.0f);  break;
        default:                      accel_psc = 0.0f;           break;
    }

#if MPU6500_ENABLE_ACCEL_BIAS
    accel_bias[0][0] = 1.0f;
    accel_bias[0][1] = 0.0f;
    accel_bias[0][2] = 0.0f;
    accel_bias[1][0] = 0.0f;
    accel_bias[1][1] = 1.0f;
    accel_bias[1][2] = 0.0f;
    accel_bias[2][0] = 0.0f;
    accel_bias[2][1] = 0.0f;
    accel_bias[2][2] = 1.0f;

    // Temp variables are needed since getData() uses bias variables.
    Vector3D temp_accel_bias;
#endif

    for (unsigned i = 0; i < BIAS_SAMPLE_COUNT; i++) {
        getData();
        temp_gyro_bias = temp_gyro_bias - gyro;
#if MPU6500_ENABLE_ACCEL_BIAS
        temp_accel_bias = temp_accel_bias + accel;
#endif
    }

    gyro_bias = temp_gyro_bias / BIAS_SAMPLE_COUNT;

#if MPU6500_ENABLE_ACCEL_BIAS
    accel_bias[2][0] = temp_accel_bias.x / BIAS_SAMPLE_COUNT;
    accel_bias[2][1] = temp_accel_bias.y / BIAS_SAMPLE_COUNT;
    accel_bias[2][2] = temp_accel_bias.z / BIAS_SAMPLE_COUNT;
    accel_bias[0][0] = accel_bias[2][1] - accel_bias[2][2];
    accel_bias[0][1] = accel_bias[2][2] - accel_bias[2][0];
    accel_bias[0][0] = accel_bias[2][0] - accel_bias[2][1];
    float length = sqrtf(accel_bias[0][0] * accel_bias[0][0] + accel_bias[0][1] * accel_bias[0][1]
                         + accel_bias[0][2] * accel_bias[0][2]);
    accel_bias[0][0] /= length;
    accel_bias[0][1] /= length;
    accel_bias[0][2] /= length;
    Vector3D temp_vect = Vector3D(accel_bias[0]).crossMultiply(Vector3D(accel_bias[2]));
    accel_bias[1][0] = temp_vect.x;
    accel_bias[1][1] = temp_vect.y;
    accel_bias[1][2] = temp_vect.z;
#endif

    // Start the update thread
    updateThread.start(prio);

    return true;
}

void MPU6500::getData() {

    // Acquire data
    uint8_t tx_data = MPU6500_ACCEL_XOUT_H | MPU6500_SPI_READ;
    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, 1, &tx_data);
    chThdSleepMilliseconds(1);
    spiReceive(&MPU6500_SPI_DRIVER, MPU6500_RX_BUF_SIZE, mpu6500_RXData);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);


    chSysLock();  // --- Enter Critical Zone ---

    accel_orig.x = accel_psc * (int16_t) ((mpu6500_RXData[0] << 8) | mpu6500_RXData[1]); // Accel X
    accel_orig.y = accel_psc * (int16_t) ((mpu6500_RXData[2] << 8) | mpu6500_RXData[3]); // Accel Y
    accel_orig.z = accel_psc * (int16_t) ((mpu6500_RXData[4] << 8) | mpu6500_RXData[5]); // Accel Z
    gyro_orig.x = gyro_psc * (int16_t) ((mpu6500_RXData[8] << 8) | mpu6500_RXData[9]);  // Gyro X
    gyro_orig.y = gyro_psc * (int16_t) ((mpu6500_RXData[10] << 8) | mpu6500_RXData[11]);  // Gyro Y
    gyro_orig.z = gyro_psc * (int16_t) ((mpu6500_RXData[12] << 8) | mpu6500_RXData[13]);  // Gyro Z
    auto temper = (int16_t) ((mpu6500_RXData[6] << 8) | mpu6500_RXData[7]); // Temperature

    temperature = (((float) temper - TEMP_OFFSET) / 333.87f) + 21.0f;

    gyro= gyro_orig + gyro_bias;
#if MPU6500_ENABLE_ACCEL_BIAS
    accel = accel_orig * accel_bias;
#endif

    last_update_time = SYSTIME;

    chSysUnlock();  // --- Exit Critical Zone ---
}

void MPU6500::UpdateThread::main() {
    setName("mpu6500");
    while (!shouldTerminate()) {

        getData();

        chSysLock();   // --- Enter Critical Zone ---

        if (ABS_IN_RANGE(gyro.x, STATIC_RANGE) &&
            ABS_IN_RANGE(gyro.y, STATIC_RANGE) &&
            ABS_IN_RANGE(gyro.z, STATIC_RANGE)) {
            static_measurement_count++;
            temp_gyro_bias = temp_gyro_bias - gyro_orig;
        } else {
            LED::red_toggle();
            static_measurement_count = 0;
            temp_gyro_bias = Vector3D(0, 0, 0);
        }

        if (static_measurement_count >= BIAS_SAMPLE_COUNT) {
            LED::green_toggle();
            gyro_bias = temp_gyro_bias / BIAS_SAMPLE_COUNT;
            static_measurement_count = 0;
            temp_gyro_bias = Vector3D(0, 0, 0);
        }

        chSysUnlock();  // --- Exit Critical Zone ---

        sleep(TIME_MS2I(THREAD_UPDATE_INTERVAL));
    }
}