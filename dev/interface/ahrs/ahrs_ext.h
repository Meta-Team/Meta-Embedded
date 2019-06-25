//
// Created by liuzikai on 2019-06-12.
//

#ifndef META_INFANTRY_AHRS_EXT_H
#define META_INFANTRY_AHRS_EXT_H

#include "ch.hpp"
#include "hal.h"

#include "ahrs_abstract.h"

#include "can_interface.h"

/**
 * @name AHRSExt
 * @brief Attitude And Heading Reference System using external AHRS module
 * @usage 1. Call start() to enable external AHRS module
 *        2. Make use of data, angle, etc.
 */
class AHRSExt : public AbstractAHRS {

public:

    /**

     (From AbstractMPU)
     Vector3D gyro;   // Data from gyroscope [deg/s]
     Vector3D accel;  // Data from accelerometer [m/s^2]
     time_msecs_t mpu_update_time = 0;  // Last update time from system start [ms]

     (From AbstractIST)
     Vector3D magnet;  // Magnet data [uT]
     time_msecs_t ist_update_time = 0;  // Last update time from system start [ms]

     (From AbstractAHRS)
     Vector3D angle;
     time_msecs_t ahrs_update_time = 0;  // Last update time from system start [ms]

    */

    /**
     * Start external AHRS update thread
     * @param can_interface initialized yaw, pitch and bullet_loader motor
     */
    void start(CANInterface *can_interface);

    AHRSExt() { inst = this; };

private:

    Vector3D gyro_orig;   // raw (biased) data of gyro
    Vector3D accel_orig;  // raw (not rotated) data from accel

    float gyro_psc;   // the coefficient converting the raw data to degree
    float accel_psc;  // the coefficient converting the raw data to m/s^2

    Vector3D gyro_bias;        // averaged gyro value when "static"
    Vector3D temp_gyro_bias;   // temp sum of gyro for calibration

    // If changes in x, y, z of gyro is in this range MPU6500 is regarded as "static"
    const float STATIC_RANGE = 0.5f;

    const unsigned BIAS_SAMPLE_COUNT = 500;
    // When static_measurement_count reaches BIAS_SAMPLE_COUNT, calibration is performed.
    unsigned static_measurement_count;
    time_msecs_t last_calibration_time;

    static constexpr float GYRO_PSC = (1.0f / 16.4f);   // coefficient converting raw data to degree, 2000 deg/s
    static constexpr float ACCEL_PSC = (1 / 16384.0f) * GRAV_CONSTANT;  //  coefficient converting raw data to m/s^2, 2g

    void update(CANRxFrame const *rxmsg);

    CANInterface *can;

    static AHRSExt* inst;  // instance pointer to allow static callback function to find the instance

    // Callback function must be static since it can't have this pointer, which is unknown to CANInterface
    static void process_feedback(CANRxFrame const *rxmsg);

    // Bits to float converter
    union conv32 {
        float fp32;
        uint32_t ui32;
    };

};


#endif //META_INFANTRY_AHRS_EXT_H
