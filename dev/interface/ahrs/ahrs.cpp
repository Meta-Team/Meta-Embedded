#include "ahrs.h"


void AHRSOnBoard::start(const Matrix33 installPosition, tprio_t mpuPrio, tprio_t istPrio, tprio_t ahrsPrio) {

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            installPos[i][j] = installPosition[i][j];
        }
    }

    MPUOnBoard::start(mpuPrio);
    ISTOnBoard::start(istPrio);

    chThdSleepMilliseconds(100);

    fetch_data();
    ::AHRS_init(q, (const fp32 *) &accel_, (const fp32 *) &mag_);

    updateThread.start(ahrsPrio);
}

Vector3D AHRSOnBoard::get_gyro() {
    return MPUOnBoard::get_gyro() * installPos;
}

Vector3D AHRSOnBoard::get_accel() {
    return MPUOnBoard::accel * installPos;
}

void AHRSOnBoard::fetch_data() {
    gyro_ = get_gyro() * DEG2RAD;
    accel_ = get_accel();
    mag_ = ISTOnBoard::get_magnet();
}

void AHRSOnBoard::update() {
    fetch_data();
    ::AHRS_update(q, 0.001f, (const fp32 *) &gyro_, (const fp32 *) &accel_, (const fp32 *) &mag_);

    ::get_angle(q, &angle.x, &angle.y, &angle.z);
    angle = angle * RAD2DEG;
}

void AHRSOnBoard::UpdateThread::main() {
    setName("ahrs");
    while (!shouldTerminate()) {
        ahrs.update();
        sleep(TIME_MS2I(CALCULATION_INTERVAL));
    }
}