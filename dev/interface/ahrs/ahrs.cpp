#include "ahrs.h"

void AHRSOnBoard::start(const Matrix33 mpu_rotation_matrix_, tprio_t update_thread_prio) {

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mpu_rotation_matrix[i][j] = mpu_rotation_matrix_[i][j];
        }
    }

    imu.start(update_thread_prio - 1);

    while(!imu.ready()) chThdSleepMilliseconds(1);

    fetch_data();
    ::AHRS_init(q, (const fp32 *) &accel, (const fp32 *) &magnet);

    chibios_rt::BaseStaticThread<512>::start(update_thread_prio);
}

void AHRSOnBoard::fetch_data() {
    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    {
        gyro_deg = mpu_rotation_matrix * imu.get_gyro();  // rotated
        gyro_rad = gyro_deg * DEG2RAD;
        accel = mpu_rotation_matrix * imu.get_accel();    // rotated
        magnet = imu.get_magnet();
    }
    chSysUnlock();  /// --- EXIT S-Locked state ---
}

void AHRSOnBoard::update() {
    fetch_data();  // should be called in NORMAL state

    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    {
        ::AHRS_update(q, 0.001f, (const fp32 *) &gyro_rad, (const fp32 *) &accel, (const fp32 *) &magnet);
        ::get_angle(q, &angle.x, &angle.y, &angle.z);
        angle = angle * RAD2DEG;
        ahrs_update_time = SYSTIME;
    }
    chSysUnlock();  /// --- EXIT S-Locked state ---
}

void AHRSOnBoard::main() {
    setName("AHRS");
    while (!shouldTerminate()) {
        update();
        sleep(TIME_MS2I(UPDATE_THREAD_INTERVAL));
    }
}