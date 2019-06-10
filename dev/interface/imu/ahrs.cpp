#include "ahrs.h"

Matrix33 AHRS::installPos;
Vector3D AHRS::angle;
Vector3D AHRS::gyro;
Vector3D AHRS::accel;
Vector3D AHRS::mag;
float AHRS::q[4];
AHRS::AHRSUpdateThread AHRS::updateThread;

void AHRS::start(const Matrix33 installPosition, tprio_t prio) {

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            installPos[i][j] = installPosition[i][j];
        }
    }

    fetch_data();
    AHRS_init(q, (const fp32 *) &accel, (const fp32 *) &mag);

    updateThread.start(prio);
}

void AHRS::fetch_data() {
    gyro = (MPU6500::gyro * installPos) * (M_PI / 180.0f);
    accel = (MPU6500::accel_orig * installPos) * 9.8f;
    mag = IST8310::magnet;
}

void AHRS::AHRSUpdateThread::main() {
    setName("ahrs");
    while (!shouldTerminate()) {

        fetch_data();
        AHRS_update(q, 0.001f, (const fp32 *) &gyro, (const fp32 *) &accel, (const fp32 *) &mag);

        get_angle(q, &angle.x, &angle.y, &angle.z);

        sleep(TIME_MS2I(AHRS_CALCULATION_INTERVAL));
    }
}