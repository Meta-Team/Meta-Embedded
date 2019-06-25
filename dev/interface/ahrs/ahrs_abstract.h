//
// Created by liuzikai on 2019-06-12.
//

#ifndef META_INFANTRY_AHRS_ABSTRACT_H
#define META_INFANTRY_AHRS_ABSTRACT_H

#include "ahrs_math.hpp"

class AbstractMPU {
public:

    /**
     * Data from gyroscope [deg/s]
     */
    Vector3D gyro;

    /**
     * Data from accelerometer [m/s^2]
     */
    Vector3D accel;

    /**
     * Last update time from system start [ms]
     */
    time_msecs_t mpu_update_time = 0;

};

class AbstractIST {
public:
    /**
     * Magnet data [uT]
     */
    Vector3D magnet;

    /**
     * Last update time from system start [ms]
     */
    time_msecs_t ist_update_time = 0;
};

class AbstractAHRS : public AbstractMPU, public AbstractIST {
public:
    /**
     * Board angle [degree]
     * @note x - yaw, y - pitch, z - roll
     */
    Vector3D angle;

    /**
     * Last update time from system start [ms]
     */
    time_msecs_t ahrs_update_time = 0;
};

#endif //META_INFANTRY_AHRS_ABSTRACT_H
