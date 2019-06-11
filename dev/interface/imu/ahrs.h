#ifndef META_ATTITUDE_CALC_H
#define META_ATTITUDE_CALC_H

#define IMU_ENABLE_IST8310  FALSE

#include "ch.hpp"
#include "hal.h"

#include "imu_math.hpp"
#include "math.h"
#include "mpu6500.h"

#if IMU_ENABLE_IST8310
#include "ist8310.h"
#endif

class AHRS {

public:

    struct quaternion_t {
        float q0, q1, q2, q3;
    };

    static quaternion_t q;

    static Vector3D angle;

    static void init(tprio_t prio);

private:

    // Local storage
    static Vector3D w; // angular speed
    static Vector3D a; // acceleration
    static Vector3D m; // magnetic field

    // Integral for pid
    static Vector3D integral;

    static void update_quaternion();

    static void fetch_data();

    static void calc_quaternion();

    // Pid constant
    static constexpr float kp = 2.0f;
    static constexpr float ki = 1.0f;
    static constexpr float kd = 0.0f;

    static constexpr unsigned dt = 1; // [ms]


    class AHRSUpdateThread : public chibios_rt::BaseStaticThread<512> {
    public:
        explicit AHRSUpdateThread(unsigned dt_) : AHRS_UPDATE_THREAD_INTERVAL(dt_) {}

    private:
        const unsigned AHRS_UPDATE_THREAD_INTERVAL;

        void main() final {
            setName("ahrs");
            while (!shouldTerminate()) {
//                chSysLock();    // --- Enter Critical Zone ---
                AHRS::update_quaternion();
//                chSysUnlock();  // --- Exit Critical Zone ---
                sleep(TIME_MS2I(AHRS_UPDATE_THREAD_INTERVAL));
            }
        }
    };

    static AHRSUpdateThread updateThread;

    float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long *) &y;
        i = 0x5f3759df - (i >> 1);
        y = *(float *) &i;
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
};

#endif // META_ATTITUDE_CALC_H