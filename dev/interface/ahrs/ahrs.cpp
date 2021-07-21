#include "ahrs.h"


void AHRSOnBoard::start(const Matrix33 mpuRotationMatrix_, tprio_t mpuPrio, tprio_t istPrio, tprio_t ahrsPrio) {

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mpuRotationMatrix[i][j] = mpuRotationMatrix_[i][j];
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
    return mpuRotationMatrix * MPUOnBoard::get_gyro();
}

Vector3D AHRSOnBoard::get_accel() {
    return mpuRotationMatrix * MPUOnBoard::get_accel();
}

void AHRSOnBoard::fetch_data() {
    gyro_ = AHRSOnBoard::get_gyro() * DEG2RAD;  // rotated
    accel_ = AHRSOnBoard::get_accel();          // rotated
    mag_ = ISTOnBoard::get_magnet();
}

bool AHRSOnBoard::AHRS_ready() {
    return MPU6500ready();
}

void AHRSOnBoard::update() {
    fetch_data();
    ::AHRS_update(q, 0.001f, (const fp32 *) &gyro_, (const fp32 *) &accel_, (const fp32 *) &mag_);

    ::get_angle(q, &angle.x, &angle.y, &angle.z);
    angle = angle * RAD2DEG;

    ahrs_update_time = SYSTIME;
}

void AHRSOnBoard::UpdateThread::main() {
    setName("AHRS");
    while (!shouldTerminate()) {
        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            ahrs.update();
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---
        sleep(TIME_MS2I(CALCULATION_INTERVAL));
    }
}