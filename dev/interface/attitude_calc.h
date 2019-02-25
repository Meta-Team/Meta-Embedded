#ifndef MATA_ATTITUDE_CALC_H
#define MATA_ATTITUDE_CALC_H

#include "mpu6500.h"

// output

typedef struct quaternion_t {
    float q0, q1, q2, q3;
};

class BoardAttitude {

public:

    static quaternion_t q;

    static void return_quaternion();
    // init pid
//    BoardAttitude(int p, int i, int d): kp(p), ki(i), kd(d), dt(0) {
//        BoardAttitude();
//    }
    
private:

    // input
    static Vector3D w; // angular speed
    static Vector3D a;
    static float a[3]; // acceleration
    static float m[3]; // magnetic field
    // TODO: assume dt are same in imu6500 and ist8310
    static float dt;

    // integral for pid
    static float integral[3];
    // pid constant
    static constexpr float kp = 1.0f;
    static constexpr float ki = 0.0f;
    static constexpr float kd = 0.0f;

    // init q, b
    BoardAttitude() {
        // init m[3]
        m[0] = m[1] = m[2] = 0;
        // init q[3]
        q[0] = 1.0f;
        q[1] = q[2] = q[3] = 0.0f;
        // init integral[3]
        integral[0] = integral[1] = integral[2] = 0.0f;
    };

    // update quaternion
    static void quaternion_update();
    static void with_ist8310();
    static void without_ist8310();
};

#endif // MATA_ATTITUDE_CALC_H