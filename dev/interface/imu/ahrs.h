#ifndef META_ATTITUDE_CALC_H
#define META_ATTITUDE_CALC_H

#include "ch.hpp"
#include "hal.h"

#include "ahrs_math.hpp"
#include "math.h"

#include "mpu6500.h"
#include "ist8310.h"
#include "ahrs_lib.h"

/**
 * @name AHRS
 * @brief Attitude And Heading Reference System using on-board IMU
 * @note This module make use of AHRS component from DJI standard program
 * @pre MPU6500 and IST8310 (WITHOUT accel rotation) have started
 * @usage 1. Call start() to enable AHRS updating thread
 *        2. Make use of data, angle, etc.
 */
class AHRS {

public:

    /**
     * Start AHRS update thread
     * @param installPosition  3x3 Matrix maps gyro and accel to desired coordinate system
     * @param prio  priority of updating thread
     */
    static void start(const Matrix33 installPosition, tprio_t prio);

    /**
     * Board angle [degree]
     */
    static Vector3D angle;

private:

    static Matrix33 installPos;

    // Local storage
    static Vector3D gyro;  // angular speed [rad/s]
    static Vector3D accel; // acceleration [m/s^2]
    static Vector3D mag;   // magnetic field [uT]

    static void fetch_data();  // helper function to fetch data from MPU6500 and IST8130

    static float q[4];

    class AHRSUpdateThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };
    static AHRSUpdateThread updateThread;
    static constexpr unsigned AHRS_CALCULATION_INTERVAL = 1; // [ms]
};

#endif // META_ATTITUDE_CALC_H