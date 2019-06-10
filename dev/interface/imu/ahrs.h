#ifndef META_ATTITUDE_CALC_H
#define META_ATTITUDE_CALC_H

#include "ch.hpp"
#include "hal.h"

#include "ahrs_math.hpp"
#include "math.h"
#include "mpu6500.h"
#include "ist8310.h"
#include "ahrs_lib.h"

class AHRS {

public:

    static void start(const Matrix33 installPosition, tprio_t prio);

    static Vector3D angle;

private:

    static Matrix33 installPos;

    // Local storage
    static Vector3D gyro;  // angular speed [rad/s]
    static Vector3D accel; // acceleration [m/s^2]
    static Vector3D mag;   // magnetic field [uT]

    static void fetch_data();

    static float q[4];

    static constexpr unsigned AHRS_CALCULATION_INTERVAL = 1; // [ms]

    class AHRSUpdateThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static AHRSUpdateThread updateThread;
};

#endif // META_ATTITUDE_CALC_H