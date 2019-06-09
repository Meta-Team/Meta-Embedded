#include "ahrs.h"

float AHRS::angle[3];
float AHRS::gyro[3];
float AHRS::accel[3];
float AHRS::mag[3];
float AHRS::q[4];
AHRS::AHRSUpdateThread AHRS::updateThread;

void AHRS::start(tprio_t prio) {
    fetch_data();
    AHRS_init(q, accel, mag);
    updateThread.start(prio);
}

void AHRS::fetch_data() {
    gyro[0] = MPU6500::angle_speed.x * M_PI / 180.0f;
    gyro[1] = MPU6500::angle_speed.y * M_PI / 180.0f;
    gyro[2] = MPU6500::angle_speed.z * M_PI / 180.0f;
    accel[0] = MPU6500::accel_orig.x * 9.8f;
    accel[1] = MPU6500::accel_orig.y * 9.8f;
    accel[2] = MPU6500::accel_orig.z * 9.8f;
    mag[0] = IST8310::magnet.x;
    mag[1] = IST8310::magnet.y;
    mag[2] = IST8310::magnet.z;
}

void AHRS::AHRSUpdateThread::main() {
    setName("ahrs");
    while (!shouldTerminate()) {

        fetch_data();
        AHRS_update(q, 0.001f, gyro, accel, mag);

        get_angle(q, &angle[0], &angle[1], &angle[2]);

        sleep(TIME_MS2I(AHRS_CALCULATION_INTERVAL));
    }
}