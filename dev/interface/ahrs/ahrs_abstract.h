//
// Created by liuzikai on 2019-06-12.
//

#ifndef META_INFANTRY_AHRS_ABSTRACT_H
#define META_INFANTRY_AHRS_ABSTRACT_H

#include "ahrs_math.hpp"

class AbstractMPU {

public:

    /**
     * Get data from gyroscope
     * @return Gyro data from gyroscope [deg/s]
     */
    virtual Vector3D get_gyro() {
        return gyro;
    }

    /**
     * Get data from accelerometer
     * @return Acceleration data from accelerometer [m/s^2]
     */
    virtual Vector3D get_accel() {
        return accel;
    }

    /**
     * Get last update time from system start
     * @return Last update time from system start [ms]
     */
    time_msecs_t get_mpu_update_time() {
        return mpu_update_time;
    }

protected:

    Vector3D gyro;  // data from gyroscope [deg/s]
    Vector3D accel;  // data from accelerometer [m/s^2]
    time_msecs_t mpu_update_time = 0;  // last update time from system start [ms]

};

class AbstractIST {

public:

    /**
     * Get magnet data from IST
     * @return Magnet data [uT]
     */
    virtual Vector3D get_magnet() {
        return magnet;
    }

    /**
     * Get last update time from system start
     * @return Last update time from system start [ms]
     */
    time_msecs_t get_ist_update_time() {
        return ist_update_time;
    }

protected:

    Vector3D magnet;  // magnet data [uT]
    time_msecs_t ist_update_time = 0;  // last update time from system start [ms]
};

// TODO: document virtual keyword here
class AbstractAHRS : virtual public AbstractMPU, virtual public AbstractIST {

public:

    /**
     * Get board angle
     * @return Board angle [degree]
     * @note x - yaw, y - pitch, z - roll
     */
    virtual Vector3D get_angle() {
        return angle;
    }

    /**
     * Get last update time from system start
     * @return Last update time from system start [ms]
     */
    time_msecs_t get_ahrs_update_time() {
        return ahrs_update_time;
    }

protected:

    Vector3D angle;  // board angle
    time_msecs_t ahrs_update_time = 0;  // last update time from system start [ms]
};

#endif //META_INFANTRY_AHRS_ABSTRACT_H
