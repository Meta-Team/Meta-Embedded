//
// Created by Tianyi Han on 3/14/2023.
//

#include "ahrs_c_interface.h"

void AHRSOnBoard_C::start(const Matrix33 imu_rotation_matrix_, const Matrix33 board_rotation_matrix_, tprio_t update_thread_prio) {

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            imu_rotation_matrix[i][j] = imu_rotation_matrix_[i][j];
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            board_rotation_matrix[i][j] = board_rotation_matrix_[i][j];
        }
    }

    imu.start(update_thread_prio - 1);
    compass.start(update_thread_prio - 2);
    chThdSleepMilliseconds(100);

    fetch_data();
    ::AHRS_init(q, (const fp32 *) &accel, (const fp32 *) &magnet);

    chibios_rt::BaseStaticThread<512>::start(update_thread_prio);
}

void AHRSOnBoard_C::fetch_data() {
     chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
     {
        gyro_rad = board_rotation_matrix * (imu_rotation_matrix * imu.get_gyro());  // rotated
        accel = board_rotation_matrix * (imu_rotation_matrix * imu.get_accel());    // rotated
        magnet = board_rotation_matrix * compass.get_compass();
     }
     chSysUnlock();  /// --- EXIT S-Locked state ---
}

void AHRSOnBoard_C::update() {
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

void AHRSOnBoard_C::main() {
    setName("AHRS");
    while (!shouldTerminate()) {
        update();
        sleep(TIME_MS2I(UPDATE_THREAD_INTERVAL));
    }
}

DEF_SHELL_CMD_START(AHRSOnBoard_C::cmdInfo)
    Shell::printf("_a:AHRS" ENDL);
    Shell::printf("_a/Angle:X{Angle} Y{Angle} Z{Angle}" ENDL);
    Shell::printf("_a/Gyro:X{Angle} Y{Angle} Z{Angle}" ENDL);
    Shell::printf("_a/Accel:X{Angle} Y{Angle} Z{Angle}" ENDL);
    Shell::printf("_a/Magnet:X{Angle} Y{Angle} Z{Angle}" ENDL);
    return true;
DEF_SHELL_CMD_END

void AHRSOnBoard_C::cmdFeedback(void *arg) {
    auto ahrs = reinterpret_cast<AHRSOnBoard_C *>(arg);
    for (int i = 0; i < 4; i++) {
        if (ahrs->feedbackEnabled[i]) {
            Vector3D v;
            switch (i) {
                case 0: v = ahrs->get_angle();  break;
                case 1: v = ahrs->get_gyro();   break;
                case 2: v = ahrs->get_accel();  break;
                case 3: v = ahrs->get_magnet(); break;
            }
            Shell::printf("_a%d %.2f %.2f %.2f" ENDL, i, v.x, v.y, v.z);
        }
    }
}

DEF_SHELL_CMD_START(AHRSOnBoard_C::cmdEnableFeedback)
    auto ahrs = reinterpret_cast<AHRSOnBoard_C *>(chp);
    int id;
    bool enabled;
    if (!Shell::parseIDAndBool(argc, argv, 4, id, enabled)) return false;
    if (id == -1) {
        for (bool &e : ahrs->feedbackEnabled) e = enabled;
    } else {
        ahrs->feedbackEnabled[id] = enabled;
    }
    return true;
DEF_SHELL_CMD_END
