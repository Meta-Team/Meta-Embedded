#ifndef META_ATTITUDE_CALC_H
#define META_ATTITUDE_CALC_H

#include "ch.hpp"
#include "hal.h"

#include "imu_math.hpp"
#include "math.h"
#include "mpu6500.h"
#include "ist8310.h"
#include "ahrs_lib.h"

class AHRS {

public:

    static void start(tprio_t prio);

    static float angle[3];

private:

    // Local storage
    static float gyro[3]; // angular speed
    static float accel[3]; // acceleration
    static float mag[3]; // magnetic field

    static void fetch_data();

    static float q[4];

    static constexpr unsigned AHRS_CALCULATION_INTERVAL = 1; // [ms]

    class AHRSUpdateThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static AHRSUpdateThread updateThread;
};

#endif // META_ATTITUDE_CALC_H