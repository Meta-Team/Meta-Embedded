#include <cmath>
#include "mpu6500.h"
#include "attitude_calc.h"

quaternion_t BoardAttitude::return_quaternion() {

    quaternion_update();
    
    // return q
    quaternion_output.q0 = q[0];
    quaternion_output.q1 = q[1];
    quaternion_output.q2 = q[2];
    quaternion_output.q3 = q[3];
    return quaternion_output;
}

void BoardAttitude::quaternion_update() {  
    // input
    w[0] = MPU6500Controller::angel_speed.x;
    w[1] = MPU6500Controller::angel_speed.y;
    w[2] = MPU6500Controller::angel_speed.z;
    a[0] = MPU6500Controller::a_component.x;
    a[1] = MPU6500Controller::a_component.y;
    a[2] = MPU6500Controller::a_component.z;

    // normalize a
    if (!(a[0] == 0.0f && a[1] == 0.0f && a[2] == 0.0f)) {
        float lena = 1.0f / sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
        a[0] *= lena;
        a[1] *= lena;
        a[2] *= lena;
    }

    //calc the third row of nRb
    float halfvx = q[1] * q[3] - q[0] * q[2];
    float halfvy = q[0] * q[1] + q[2] * q[3];
    float halfvz = q[0] * q[0] + q[3] * q[3] - 0.5f;

    // calc error
    float halfex = a[1] * halfvz - a[2] * halfvy;
    float halfey = a[2] * halfvx - a[0] * halfvz;
    float halfez = a[0] * halfvy - a[1] * halfvx;

    // handle ki
    if (ki > 0.0f) {
        integral[0] = ki * 2.0f * halfex * MPU6500Controller::dt;
        integral[1] = ki * 2.0f * halfey * MPU6500Controller::dt;
        integral[2] = ki * 2.0f * halfez * MPU6500Controller::dt;

        w[0] += integral[0];
        w[1] += integral[1];
        w[2] += integral[2];
    } else {
        integral[0] = integral[1] = integral[2] = 0.0f;
    }

    // correct w using kp
    w[0] += kp * 2 * halfex;
    w[1] += kp * 2 * halfey;
    w[2] += kp * 2 * halfez;
    w[0] *= 0.5f * MPU6500Controller::dt;
    w[1] *= 0.5f * MPU6500Controller::dt;
    w[2] *= 0.5f * MPU6500Controller::dt;

    // update q
    float qa = q[0];
    float qb = q[1];
    float qc = q[2]; 
    q[0] += (- qb * w[0] - qc * w[1] - q[3] * w[2]);
    q[1] += (  qa * w[0] + qc * w[2] - q[3] * w[1]);
    q[2] += (  qa * w[1] - qb * w[2] + q[3] * w[0]);
    q[3] += (  qa * w[2] + qb * w[1] - qc   * w[0]);

    // normalize q
    if (!(q[0] == 0.0f && q[1] == 0.0f && q[2] == 0.0f && q[3] == 0.0f)) {
        float lenq = 1 / sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        q[0] *= lenq;
        q[1] *= lenq;
        q[2] *= lenq;
        q[3] *= lenq;
    }
 }
