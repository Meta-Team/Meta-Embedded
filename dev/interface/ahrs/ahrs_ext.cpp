//
// Created by liuzikai on 2019-06-12.
//

#include "ahrs_ext.h"

AHRSExt *AHRSExt::inst;

void AHRSExt::load_calibration_data(Vector3D gyro_bias_) {
    gyro_bias = gyro_bias_;
    last_calibration_time = SYSTIME;
}

void AHRSExt::start(CANInterface *can_interface) {
    can = can_interface;
    can->register_callback(0x0A, 0x0C, process_feedback);

    // Wait for first calibration
    while (last_calibration_time == 0) {
        chThdSleepMilliseconds(1);
    }
}

void AHRSExt::update(CANRxFrame const *rxmsg) {

    // Fetch data from CAN
    if (rxmsg->SID == 0x0A) {
        angle.y = ((union conv32) {.ui32 = rxmsg->data32[0]}).fp32;  // pitch
        angle.z = ((union conv32) {.ui32 = rxmsg->data32[1]}).fp32;  // roll
    } else if (rxmsg->SID == 0x0B) {
        angle.x = ((union conv32) {.ui32 = rxmsg->data32[0]}).fp32;  // yaw
        accel_orig.x = ACCEL_PSC * (int16_t) rxmsg->data16[2];
        accel_orig.y = ACCEL_PSC * (int16_t) rxmsg->data16[3];
    } else if (rxmsg->SID == 0x0C) {
        accel_orig.z = ACCEL_PSC * (int16_t) rxmsg->data16[0];


        // Perform gyro process only after final package

        Vector3D new_gyro_orig = Vector3D((int16_t) rxmsg->data16[1],
                                          (int16_t) rxmsg->data16[2],
                                          (int16_t) rxmsg->data16[3]) * GYRO_PSC;

        if (ABS_IN_RANGE(new_gyro_orig.x - gyro_orig.x, STATIC_RANGE) &&
            ABS_IN_RANGE(new_gyro_orig.y - gyro_orig.y, STATIC_RANGE) &&
            ABS_IN_RANGE(new_gyro_orig.z - gyro_orig.z, STATIC_RANGE)) {
            static_measurement_count++;
            temp_gyro_bias = temp_gyro_bias - new_gyro_orig;
        } else {
            static_measurement_count = 0;
            temp_gyro_bias = Vector3D(0, 0, 0);
        }

        gyro_orig = new_gyro_orig;
        gyro = gyro_orig + gyro_bias;
        accel = accel_orig;

        mpu_update_time = ist_update_time = ahrs_update_time = SYSTIME;

        if (static_measurement_count >= BIAS_SAMPLE_COUNT) {
            gyro_bias = temp_gyro_bias / BIAS_SAMPLE_COUNT;
            static_measurement_count = 0;
            temp_gyro_bias = Vector3D(0, 0, 0);
            last_calibration_time = SYSTIME;
        }

    }

}

void AHRSExt::process_feedback(CANRxFrame const *rxmsg) {
    inst->update(rxmsg);
}