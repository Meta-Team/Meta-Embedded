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

};

class AbstractIST {
public:
    /**
     * Magnet data [uT]
     */
    Vector3D magnet;
};

class AbstractAHRS : public AbstractMPU, public AbstractIST {
public:
    /**
     * Board angle [degree]
     * @note x - yaw, y - pitch, z - roll
     */
    Vector3D angle;
};

#endif //META_INFANTRY_AHRS_ABSTRACT_H
