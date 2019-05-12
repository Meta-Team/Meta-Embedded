#ifndef META_ATTITUDE_CALC_H
#define META_ATTITUDE_CALC_H

#include "mpu6500.h"

// output
typedef struct quaternion_t {
    float q0, q1, q2, q3;
};

class BoardAttitude {
    
public:

    static quaternion_t q;
    static void return_quaternion();
    static void attitude_init(float _kp, float _ki, float _kd) {
        // init q
        q.q0 = 1.0f;
        q.q1 = q.q2 = q.q3 = 0.0f; 
        // init m
        m.x = m.y = m.z = 0;  
        // init integral
        integral.x = integral.y = integral.z = 0.0f;
        // init pid constant
        kp = _kp;
        ki = _ki;
        kd = _kd;
    }
    
private:

    // input
    static Vector3D w; // angular speed
    static Vector3D a; // acceleration
    static Vector3D m; // magnetic field

    // TODO: assume dt are same in imu6500 and ist8310
    static float dt;

    // integral for pid
    static Vector3D integral;

    // pid constant
    static float kp;
    static float ki;
    static float kd;

    // update quaternion
    static void quaternion_update();
    static void with_ist8310();
    static void without_ist8310();
};

#endif // META_ATTITUDE_CALC_H