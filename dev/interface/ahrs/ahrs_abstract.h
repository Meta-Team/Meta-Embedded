//
// Created by liuzikai on 2019-06-12.
//

#ifndef META_INFANTRY_AHRS_ABSTRACT_H
#define META_INFANTRY_AHRS_ABSTRACT_H

#include "ahrs_math.hpp"

class AbstractAHRS {
public:

    /**
     * Get data from gyroscope
     * @return Gyro data from gyroscope [deg/s]
     */
    virtual Vector3D get_gyro() = 0;

    /**
     * Get data from accelerometer
     * @return Acceleration data from accelerometer [m/s^2]
     */
    virtual Vector3D get_accel() = 0;

    /**
     * Get last update time from system start
     * @return Last update time from system start [ms]
     */
    time_msecs_t get_mpu_update_time() const {
        return mpu_update_time;
    }

    /**
     * Get magnet data from IST
     * @return Magnet data [uT]
     */
    virtual Vector3D get_magnet() = 0;

    /**
     * Get last update time from system start
     * @return Last update time from system start [ms]
     */
    time_msecs_t get_ist_update_time() const {
        return ist_update_time;
    }

    /**
     * Get board angle
     * @return Board angle [degree]
     * @note x - yaw, y - pitch, z - roll
     */
    virtual Vector3D get_angle() = 0;

    /**
     * Get last update time from system start
     * @return Last update time from system start [ms]
     */
    time_msecs_t get_ahrs_update_time() const {
        return ahrs_update_time;
    }

protected:

    time_msecs_t mpu_update_time = 0;   // last update time from system start [ms]
    time_msecs_t ist_update_time = 0;   // last update time from system start [ms]
    time_msecs_t ahrs_update_time = 0;  // last update time from system start [ms]
};

#endif //META_INFANTRY_AHRS_ABSTRACT_H
